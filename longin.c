/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3.0 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTAlonginLITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) Abdallah Ismail <abdallah.ismail@sesame.org.jo>, 2015
 */

/*
 * @file 	longin.c
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	11/9/2015
 * @brief	Implements epics device support layer for the PMC-evg230 event receiver
 */

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
#include <longinRecord.h>

/*Application includes*/
#include "parse.h"
#include "evg.h"

/*Macros*/
#define NUMBER_OF_IO	100

/*Local variables*/
static	io_t		io[NUMBER_OF_IO];
static	uint32_t	ioCount;

/*Function prototypes*/
static	long	init		(int after);
static	long	initRecord	(longinRecord *record);
static 	long	ioRecord	(longinRecord *record);
static	void*	thread		(void* arg);

/*Function definitions*/

/** 
 * @brief 	Initializes record count
 *
 * @return	0 on success, -1 on failure
 */
static long
init(int after)
{
	if (!after)
		ioCount = 0;
	return 0;
}

/** 
 * @brief 	Initializes the record
 *
 * This function is called by recordInit during IOC initialization.
 * For each record of this type, this function attemps the following:
 * 	Checks record parameters.
 * 	Parses record parameters.
 * 	Sets record's private structure.
 *
 * @param	record	:	Pointer to record being initialized.
 * @return	0 on success, -1 on failure.
 */
static long 
initRecord(longinRecord *record)
{
	int32_t	status;

	if (ioCount >= NUMBER_OF_IO)
	{
		printf("[evg][initRecord] Unable to initialize %s: Too many records\r\n", record->name);
		return -1;
	}
	if (record->inp.type != INST_IO) 
	{
		printf("[evg][initRecord] Unable to initialize %s: Illegal io type\r\n", record->name);
		return -1;
	}

	status			=	parse(&io[ioCount], record->inp.value.instio.string);
	if (status < 0)
	{
		printf("[evg][initRecord] Unable to initialize %s: Could not parse parameters\r\n", record->name);
		return -1;
	}

	io[ioCount].device	=	evg_open(io[ioCount].name);	
	if (io[ioCount].device == NULL)
	{
		printf("[evg][initRecord] Unable to initalize %s: Could not open device\r\n", record->name);
		return -1;
	}

	record->dpvt	=	&io[ioCount];
	ioCount++;

	return 0;
}

/** 
 * @brief 	Performs IO on the record.
 *
 * This function is called by record support to perform IO on the record
 * This function attemps the following:
 * 	Checks record parameters.
 * 	Executes record IO.
 *
 * @param	record	:	Pointer to record being initialized.
 * @return	0 on success, -1 on failure.
 */
static long 
ioRecord(longinRecord *record)
{
	int32_t		status	=	0;
	io_t*		private	=	(io_t*)record->dpvt;
	pthread_t	handle;

	if (!record)
	{
		printf("[evg][ioRecord] Unable to perform io on %s: Null record pointer\r\n", record->name);
		return -1;
	}
    if (!private)
    {
        printf("[evg][ioRecord] Unable to perform io on %s: Null private structure pointer\r\n", record->name);
        return -1;
    }
	if (!private->command || !strlen(private->command))
	{
		printf("[evg][ioRecord] Unable to perform io on %s: Command is null or empty\r\n", record->name);
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
			printf("[evg][ioRecord] Unable to perform IO on %s: Unable to create thread\r\n", record->name);
			return -1;
		}
		record->pact = true;
		return 0;
	}

	/*
	 * This is the second pass, complete the request and return
	 */
	if (private->status	< 0)
	{
		printf("[evg][ioRecord] Unable to perform IO on %s\r\n", record->name);
		record->pact=	false;
		return -1;
	}
	record->pact	=	false;

	return 0;
}

/** 
 * @brief 	Performs asynchronousIO on the record
 *
 * This function attemps the following:
 * 	Detaches the thread
 * 	Performs the requested IO
 *	Processes the record to finalize IO
 *
 * @param	arg	:	Pointer to thread arguments
 * @return	0 on success, -1 on failure
 */
void*
thread(void* arg)
{
	uint8_t		byte;
	uint16_t	word;
	int			status	=	0;
	longinRecord*	record	=	(longinRecord*)arg;
	io_t*		private	=	(io_t*)record->dpvt;

	/*Detach thread*/
	pthread_detach(pthread_self());

	if (strcmp(private->command, "setEvent") == 0)
	{
		status	=	evg_getEvent(private->device, private->sequencer, private->address, &byte);
		if (status < 0)
		{
			printf("[evg][thread] Unable to io %s\r\n", record->name);
			private->status	=	-1;
		}
		record->val	=	byte;
	}
	else if (strcmp(private->command, "getRfPrescaler") == 0)
	{
		status	=	evg_getRfPrescaler(private->device, &byte);
		if (status < 0)
		{
			printf("[evg][thread] Unable to io %s\r\n", record->name);
			private->status	=	-1;
		}
		record->val	=	byte;
	}
	else if (strcmp(private->command, "getAcPrescaler") == 0)
	{
		status	=	evg_getAcPrescaler(private->device, &byte);
		if (status < 0)
		{
			printf("[evg][thread] Unable to io %s\r\n", record->name);
			private->status	=	-1;
		}
		record->val	=	byte;
	}
	else if (strcmp(private->command, "getSequencerPrescaler") == 0)
	{
		status	=	evg_getSequencerPrescaler(private->device, private->sequencer, &word);
		if (status < 0)
		{
			printf("[evg][thread] Unable to io %s\r\n", record->name);
			private->status	=	-1;
		}
		record->val	=	word;
	}
	else
	{
		printf("[evg][thread] Unable to io %s: Do not know how to process \"%s\" requested by %s\r\n", record->name, private->command, record->name);
		private->status	=	-1;
	}

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
} longinevg =
{
    5,
    NULL,
    init,
    initRecord,
    NULL,
    ioRecord
};
epicsExportAddress(dset, longinevg);
