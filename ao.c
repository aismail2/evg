/*Standard includes*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

/*EPICS includes*/
#include <epicsExport.h>
#include <devSup.h>
#include <errlog.h>
#include <dbAccess.h>
#include <recSup.h>
#include <aoRecord.h>

/*Application includes*/
#include "evg.h"

/*Macros*/
#define NUMBER_OF_outputS	100
#define COMMAND_LENGTH		40

typedef struct
{
	void*	device;
	char	name	[NAME_LENGTH];	
	char	command	[COMMAND_LENGTH];
} output_t;

/*Local variables*/
static	output_t	outputs[NUMBER_OF_outputS];
static	uint32_t	outputCount;

/*Function prototypes*/
static	long	init(int after);
static	long	initRecord(aoRecord *record);
static 	long	writeRecord(aoRecord *record);
static	void*	thread(void* arg);

/*Function definitions*/
static long
init(int after)
{
	if (!after)
		outputCount = 0;
	return 0;
}

static long 
initRecord(aoRecord *record)
{
	uint32_t	i;
	char		*parameters;
	char		*token;
	char		tokens[][COMMAND_LENGTH]	=	{"", "", "", "", ""};

	if (outputCount >= NUMBER_OF_outputS)
	{
		errlogPrintf("Unable to initialize %s: Too many records\r\n", record->name);
		return -1;
	}

    if (record->out.type != INST_IO) 
	{
		errlogPrintf("Unable to initialize %s: Illegal output type\r\n", record->name);
		return -1;
	}

	/*
	 * Parse output
	 */
	parameters		=	record->out.value.instio.string;

	/*Collect tokens*/
	token	=	strtok(parameters, " ");
	if (token)
		strcpy(tokens[0], token);
	for (i = 1; token != NULL; i++)
	{
		token	=	strtok(NULL, " ");
		if (token)
			strcpy(tokens[i], token);
	}

	/*Parse name*/
	token	=	strtok(tokens[0], ":");
	if (!token)
	{
		errlogPrintf("Unable to initialize %s: Missing device name\r\n", record->name);
		return -1;
	}
	strcpy(outputs[outputCount].name, token);

	/*Parse command*/
	token	=	strtok(NULL, "");
	if (!token)
	{
		errlogPrintf("Unable to initialize %s: Missing command\r\n", record->name);
		return -1;
	}
	strcpy(outputs[outputCount].command, token);

	outputs[outputCount].device	=	evg_open(outputs[outputCount].name);	
	if (outputs[outputCount].device < 0)
	{
		errlogPrintf("Unable to initalize %s: Could not open device\r\n", record->name);
		return -1;
	}

	record->dpvt				=	&outputs[outputCount];
	outputCount++;

	return 0;
}

static long 
writeRecord(aoRecord *record)
{
	int			status;
	output_t*	private	=	(output_t*)record->dpvt;
	pthread_t	handle;


	if (!record)
	{
		errlogPrintf("Unable to read %s: Null record pointer\r\n", record->name);
		return -1;
	}

    if (!private)
    {
        errlogPrintf("Unable to read %s: Null private structure pointer\r\n", record->name);
        return -1;
    }

	if (!private->command || !strlen(private->command))
	{
		errlogPrintf("Unable to read %s: Command is null or empty\r\n", record->name);
		return -1;
	}

	/*
	 * Start IO
	 */

	/*If this is the first pass then start IO thread, set PACT, and return*/
	if(!record->pact)
	{
		status	=	pthread_create(&handle, NULL, thread, (void*)record);	
		if (status)
		{
			errlogPrintf("Unable to read %s: Unable to create thread\r\n", record->name);
			return -1;
		}
		record->pact = true;
		return 0;
	}

	/*
	 * This is the second pass, complete the request and return
	 */
	record->pact	=	false;

	return 0;
}

void*
thread(void* arg)
{
	int			status	=	0;
	aoRecord*	record	=	(aoRecord*)arg;
	output_t*	private	=	(output_t*)record->dpvt;

	/*Detach thread*/
	pthread_detach(pthread_self());

	if (strcmp(private->command, "setTimestamp") == 0)
		status	=	evg_setTimestamp(private->device, record->val);
	else
		errlogPrintf("Unable to read %s: Do not know how to process \"%s\" requested by %s\r\n", record->name, private->command, record->name);
	if (status < 0)
		errlogPrintf("Unable to read %s: Driver thread is unable to read\r\n", record->name);

	/*Process record*/
	dbScanLock((struct dbCommon*)record);
	(record->rset->process)(record);
	dbScanUnlock((struct dbCommon*)record);

	return NULL;
}

struct devsup {
    long	  number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN io;
} aoevg =
{
    6,
    NULL,
    init,
    initRecord,
    NULL,
    writeRecord,
	NULL
};
epicsExportAddress(dset, aoevg);
