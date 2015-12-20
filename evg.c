/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3.0 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) Abdallah Ismail <abdallah.ismail@sesame.org.jo>, 2015
 */

/*
 * @file 	evg.c
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	2014-10-09
 * @brief	Implements epics driver support layer for the VME-evg-230/RF timing card
 */

/*Standard headers*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

/*EPICS headers*/
#include <epicsExport.h>
#include <drvSup.h>
#include <errlog.h>
#include <iocsh.h>

/*Application headers*/
#include "evg.h"

/*
 * Macros
 */

/** @brief device_t is a structure that holds configured device information */
typedef struct
{
	char			name[NAME_LENGTH];	/*Device name*/
	in_addr_t		ip;					/*Device IP in network byte-order*/
	in_port_t		port;				/*Device port in network byte-order*/
	uint32_t		frequency;			/*Device event frequency in MHz*/
	pthread_mutex_t	mutex;				/*Mutex for accessing the device*/
	int32_t			socket;				/*Socket for communicating with the device*/
} device_t;

/** @brif message_t is a structure that represents the UDP message sent/received to/from the device*/
typedef struct
{
	uint8_t		access;		/*Read/Write*/
	uint8_t		status;		/*Filled by device*/
	uint16_t	data;		/*Register data*/
	uint32_t	address;	/*Register address*/
	uint32_t	reference;	/*Reserved*/
} message_t;

#define NUMBER_OF_DEVICES	10	/*Maximum number of devices allowed*/
#define NUMBER_OF_RETRIES	3	/*Maximum number of retransmissions*/

/*
 * Private members
 */
static	device_t	devices[NUMBER_OF_DEVICES];	/*Configured devices*/
static	uint32_t	deviceCount	=	0;			/*Number of configured devices*/

/*
 * Private function prototypes
 */
/*Initializes the device*/
static	long	init		(void);
/*Reports on all configured devices*/
static	long	report		(int detail);
/*Writes data and checks that it was written*/
static	long	writecheck	(void *dev, evgregister_t reg, uint16_t data);
/*Writes data to register*/
static	long	writereg	(void *dev, evgregister_t reg, uint16_t data);
/*Reads data from register*/
static	long	readreg		(void *dev, evgregister_t reg, uint16_t *data);

/*
 * Function definitions
 */

/** 
 * @brief	Searches for device with given name and returns a pointer to it 
 * @return	Void pointer to found device, NULL otherwise
 */
void*
evg_open(char *name)
{
	uint32_t	i;

	if (!name || !strlen(name) || strlen(name) >= NAME_LENGTH)
	{
		errlogPrintf("\x1B[31mCould not find device\r\n\x1B[0m");
		return NULL;
	}

	for (i = 0; i < deviceCount; i++)
	{
		if (strcmp(devices[i].name, name) == 0)
			return &devices[i];
	}
	return NULL;
}

/** 
 * @brief 	Initializes all configured devices
 *
 * This function is called by iocInit during IOC initialization.
 * For each configured device, this function attemps the following:
 *	Initialize mutex
 *	Create and bind UDP socket
 *	Disable the device
 *	Disable the sequencer and set its prescaler to 1
 *	Disable the ac trigger and set its prescaler to 50
 *	Set RF prescaler to 4
 *	Disable all outputs and reset their polarity
 *	Set all events to 0x7f (end-event) and timestamps to 0
 *
 * @return	0 on success, -1 on failure
 */
static long 
init(void)
{
	int32_t				status;			
	uint32_t			device;
	struct sockaddr_in	address;

	/*Initialize devices*/
	for (device = 0; device < deviceCount; device++)
	{
		/*Initialize mutex*/
		pthread_mutex_init(&devices[device].mutex, NULL);

		/*Create and initialize UDP socket*/
		devices[device].socket 	=	socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (devices[device].socket < 0)
		{
			errlogPrintf("\x1B[31mUnable to create socket\n\x1B[0m");
			return -1;
		}
		memset((uint8_t *)&address, 0, sizeof(address));
		address.sin_family		= 	AF_INET;
		address.sin_port 		= 	devices[device].port;
		address.sin_addr.s_addr	=	devices[device].ip;
		status	=	connect(devices[device].socket, (struct sockaddr*)&address, sizeof(address));
		if (status	<	0)
		{
			errlogPrintf("\x1B[31mUnable to connect to device\n\x1B[0m");
			return -1;
		}

		/*
		 * Initialize the device
		 */

		/*Disable the device*/
		status	=	evg_enable(&devices[device], 0);
		if (status < 0)
		{
			printf("\x1B[31m[evg][init] Cannot enable device\n\x1B[0m");
			return -1;
		}
	}
	return 0;
}

long
evg_enable(void* dev, bool enable)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Act*/
	if (enable)
	{
		status	=	writereg(device, REGISTER_CONTROL, CONTROL_ENABLE);
		if (status < 0)
		{
			printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writereg(device, REGISTER_CONTROL, CONTROL_DISABLE);
		if (status < 0)
		{
			printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_isEnabled(void* dev)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	status	=	readreg(device, REGISTER_CONTROL, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return (!(data&CONTROL_DISABLE));
}

long
evg_setRfClockSource(void* dev, rfsource_t source)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_RF_CONTROL, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Act*/
	switch (source)
	{
		case RF_SOURCE_INTERNAL:
			data	&=	~RF_CONTROL_EXTERNAL;
			status	=	writecheck(device, REGISTER_RF_CONTROL, data);
			if (status < 0)
			{
				printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
				pthread_mutex_unlock(&device->mutex);
				return -1;
			}
			break;
		default:
			status	=	writecheck(device, REGISTER_RF_CONTROL, data | RF_CONTROL_EXTERNAL);
			if (status < 0)
			{
				printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
				pthread_mutex_unlock(&device->mutex);
				return -1;
			}
			break;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getRfClockSource(void* dev, rfsource_t *source)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev || !source)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_RF_CONTROL, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	*source	=	(rfsource_t)(data&RF_CONTROL_EXTERNAL);

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setRfPrescaler(void* dev, uint8_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (prescaler >= 32)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_RF_CONTROL, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	data	&=	~RF_CONTROL_DIVIDER_MASK;

	/*Set and enable RF clock*/
	status	=	writecheck(device, REGISTER_RF_CONTROL, data | (prescaler-1));
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getRfPrescaler(void* dev, uint8_t *prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev || !prescaler)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_RF_CONTROL, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	data		&=	RF_CONTROL_DIVIDER_MASK;
	*prescaler	=	data;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setAcPrescaler(void* dev, uint8_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Read original value of register*/
	status		=	readreg(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetAcTriggerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	data	&=	~AC_ENABLE_DIVIDER_MASK;

	/*Act*/
	status	=	writecheck(device, REGISTER_AC_ENABLE, data|prescaler);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetAcPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getAcPrescaler(void* dev, uint8_t *prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Read original value of register*/
	status		=	readreg(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetAcTriggerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	data		&=	AC_ENABLE_DIVIDER_MASK;
	*prescaler	=	data;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setAcSyncSource(void* dev, acsource_t source)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Act*/
	switch (source)
	{
		case AC_SOURCE_MXC7:
			status	=	writecheck(device, REGISTER_RF_CONTROL, data | AC_ENABLE_SYNC);
			if (status < 0)
			{
				printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
				pthread_mutex_unlock(&device->mutex);
				return -1;
			}
			break;
		default:
			data	&=	~AC_ENABLE_SYNC;
			status	=	writecheck(device, REGISTER_AC_ENABLE, data);
			if (status < 0)
			{
				printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
				pthread_mutex_unlock(&device->mutex);
				return -1;
			}
			break;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getAcSyncSource(void* dev, acsource_t *source)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev || !source)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read register*/
	status	=	readreg(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	*source	=	(acsource_t)(data&AC_SOURCE_MXC7);

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_enableSequencer(void* dev, uint8_t sequencer, bool enable)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read event enable register*/
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &data);
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	switch (sequencer)
	{
		case 0:
			if (enable)
				data	|=	EVENT_ENABLE_SEQUENCER0;
			else
				data	&=	~EVENT_ENABLE_SEQUENCER0;
			break;
		default:
			if (enable)
				data	|=	EVENT_ENABLE_SEQUENCER1;
			else
				data	&=	~EVENT_ENABLE_SEQUENCER1;
			break;
	}

	/*Act*/
	status	=	writereg(device, REGISTER_EVENT_ENABLE, data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_isSequencerEnabled(void* dev, uint8_t sequencer)
{
	uint16_t	data;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read event enable register*/
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	if (sequencer)
		data	&=	EVENT_ENABLE_SEQUENCER1;
	else 
		data	&=	EVENT_ENABLE_SEQUENCER0;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return data;
}

long
evg_setSequencerTriggerSource(void* dev, uint8_t sequencer, triggersource_t source)
{
	uint16_t	enable, ac;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read registers*/
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &enable);
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &ac);
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	switch (source)
	{
		case TRIGGER_SOFT:
			enable	|=	EVENT_ENABLE_VME;
			if (sequencer)
				ac		&=	~AC_ENABLE_SEQ1;
			else 
				ac		&=	~AC_ENABLE_SEQ0;
			break;
		default:
			enable	&=	~EVENT_ENABLE_VME;
			if (sequencer)
				ac		|=	AC_ENABLE_SEQ1;
			else 
				ac		|=	AC_ENABLE_SEQ0;
			break;
	}

	/*Write registers*/
	status	=	writereg(device, REGISTER_EVENT_ENABLE, enable);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	writereg(device, REGISTER_AC_ENABLE, ac);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getSequencerTriggerSource(void* dev, uint8_t sequencer, triggersource_t *source)
{
	uint16_t	enable, ac;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read registers*/
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &enable);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	readreg(device, REGISTER_EVENT_ENABLE, &ac);
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	switch (sequencer)
	{
		case 0:
			if (ac & AC_ENABLE_SEQ0)
				*source	=	TRIGGER_AC;
			else
				*source	=	TRIGGER_SOFT;
			break;
		default:
			if (ac & AC_ENABLE_SEQ1)
				*source	=	TRIGGER_AC;
			else
				*source	=	TRIGGER_SOFT;
			break;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setSequencerPrescaler(void* dev, uint8_t sequencer, uint16_t prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Set event frequency*/
	if (sequencer)
	{
		status	=	writecheck(device, REGISTER_SEQ_CLOCK_SEL2, prescaler);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writecheck(device, REGISTER_SEQ_CLOCK_SEL1, prescaler);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getSequencerPrescaler(void* dev, uint8_t sequencer, uint16_t *prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Read event frequency*/
	if (sequencer)
	{
		status	=	readreg(device, REGISTER_SEQ_CLOCK_SEL2, prescaler);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	readreg(device, REGISTER_SEQ_CLOCK_SEL1, prescaler);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_triggerSequencer(void* dev, uint8_t sequencer)
{
	uint16_t	control;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read registers*/
	status	=	readreg(device, REGISTER_CONTROL, &control);
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	if (sequencer)
		control	|=	CONTROL_VTRG2;
	else 
		control	|=	CONTROL_VTRG1;

	/*Write registers*/
	status	=	writereg(device, REGISTER_CONTROL, control);
	if (status < 0)
	{
		printf("\x1B[31m[evr][enable] Couldn't write to control register\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setEvent(void* dev, uint8_t sequencer, uint16_t address, uint8_t event)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (address >= NUMBER_OF_ADDRESSES)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Set address*/
	if (sequencer)
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS1, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Set event*/
		status	=	writecheck(device, REGISTER_SEQ_CODE1, event);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS0, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Set event*/
		status	=	writecheck(device, REGISTER_SEQ_CODE0, event);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getEvent(void* dev, uint8_t sequencer, uint16_t address, uint8_t *event)
{
	uint16_t	readback;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (address >= NUMBER_OF_ADDRESSES)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (!event)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Set address*/
	if (sequencer)
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS1, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Read event*/
		status	=	readreg(device, REGISTER_SEQ_CODE1, &readback);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS0, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Read event*/
		status	=	readreg(device, REGISTER_SEQ_CODE0, &readback);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	*event	=	readback;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_setTimestamp(void* dev, uint8_t sequencer, uint16_t address, float timestamp)
{
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (address >= NUMBER_OF_ADDRESSES)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (timestamp > (UINT_MAX/(float)device->frequency))
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful: timestamp is too long\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert timestamp*/
	cycles	=	(uint32_t)timestamp*device->frequency;	

	/*Set address*/
	if (sequencer)
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS1, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Write new timestamp*/
		status	=	writecheck(device, REGISTER_SEQ_TIME1, cycles>>16);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestampe is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		status	=	writecheck(device, REGISTER_SEQ_TIME1+2, cycles);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS0, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Write new timestamp*/
		status	=	writecheck(device, REGISTER_SEQ_TIME0, cycles>>16);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestampe is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		status	=	writecheck(device, REGISTER_SEQ_TIME0+2, cycles);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evg_getTimestamp(void* dev, uint8_t sequencer, uint16_t address, float *timestamp)
{
	uint16_t	data	=	0;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (!dev)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (sequencer >= NUMBER_OF_SEQUENCERS)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (address >= NUMBER_OF_ADDRESSES)
	{
		printf("\x1B[31m[evr][enable] Null pointer to device\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (!timestamp)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful: timestamp is too long\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Set address*/
	if (sequencer)
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS1, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Read timestamp*/
		status	=	readreg(device, REGISTER_SEQ_TIME1, &data);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestampe is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		cycles	=	data << 16;

		status	=	readreg(device, REGISTER_SEQ_TIME1+2, &data);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		cycles	|=	data;
	}
	else
	{
		status	=	writecheck(device, REGISTER_SEQ_ADDRESS0, address);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}

		/*Read timestamp*/
		status	=	readreg(device, REGISTER_SEQ_TIME0, &data);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestampe is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		cycles	=	data << 16;

		status	=	readreg(device, REGISTER_SEQ_TIME0+2, &data);
		if (status < 0)
		{
			errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		cycles	|=	data;
	}

	/*Convert cycles to timestamp*/
	*timestamp	=	cycles/(float)device->frequency;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Writes device's 16-bit register and checks the register was written
 *
 * Prepares UDP message, sends it to device
 * Reads back the value in the register and validates the write
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	reg		:	Address of register to be written
 * @param	data	:	16-bit data to be written to device
 * @return	0 on success, -1 on failure
 */
static long	
writecheck(void *dev, evgregister_t reg, uint16_t data)
{
	uint16_t	readback;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Check inputs*/
	if (!dev)
		return -1;

	/*Write data*/
	status	=	writereg(device, reg, data);
	if (status < 0)
		return -1;

	/*Check that data was updated*/
	status	=	readreg(device, reg, &readback);
	if (status < 0)
		return -1;

	if (readback != data)
		return -1;

	return 0;
}

/**
 * @brief	Reads 16-bit register from device
 *
 * Prepares UDP message, sends it to device, and reads back reply.
 * Times out and retransmits upon detecting any communication failures.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	reg		:	Address of register to be read
 * @param	*data	:	16-bit data read from register
 * @return	0 on success, -1 on failure
 */
static long
readreg(void *dev, evgregister_t reg, uint16_t *data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

	/*Check inputs*/
	if (!dev || !data)
		return -1;

	/*Prepare message*/
	message.access		=	ACCESS_READ;
	message.status		=	0;
	message.data		=	0x0000;
	message.address		=	htonl(REGISTER_BASE_ADDRESS + reg);
	message.reference	=	0x00000000;

	for (retries = 0; retries < NUMBER_OF_RETRIES; retries++)
	{
		/*Write to device*/
		status	=	write(device->socket, &message, sizeof(message));
		if (status == sizeof(message))
		{
			/*Prepare poll structure*/
			events[0].fd		=	device->socket;	
			events[0].events	=	POLLIN;
			events[0].revents	=	0;

			/*Poll*/
			status	=	poll(events, 1, 1000);
			if (status > 0) /*Ready to read (no errors or timeouts)*/
			{
				/*Read from device*/
				status	=	read(device->socket, &message, sizeof(message));
				if (status == sizeof(message))
					break;
			}
		}
	}

	if (retries >= NUMBER_OF_RETRIES)
		return -1;

	/*Extract data*/
	*data	=	ntohs(message.data);

	return 0;
}

/**
 * @brief	Writes device's 16-bit register
 *
 * Prepares UDP message, sends it to device, and reads back reply.
 * Times out and retransmits upon detecting any communication failures.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	reg		:	Address of register to be read
 * @param	data	:	16-bit data to be written to device
 * @return	0 on success, -1 on failure
 */
static long
writereg(void *dev, evgregister_t reg, uint16_t data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

	if (!dev)
		return -1;

	/*Prepare message*/
	message.access		=	ACCESS_WRITE;
	message.status		=	0;
	message.data		=	htons(data);
	message.address		=	htonl(REGISTER_BASE_ADDRESS + reg);
	message.reference	=	0x00000000;

	/*Transmit and retry*/
	for (retries = 0; retries < NUMBER_OF_RETRIES; retries++)
	{
		/*Write to device*/
		status	=	write(device->socket, &message, sizeof(message));
		if (status == sizeof(message))
		{
			/*Prepare poll structure*/
			events[0].fd		=	device->socket;	
			events[0].events	=	POLLIN;
			events[0].revents	=	0;

			/*Poll*/
			status	=	poll(events, 1, 1000);
			if (status > 0)
			{
				/*Read from device*/
				status	=	read(device->socket, &message, sizeof(message));
				if (status == sizeof(message))
					break;
			}
		}
	}

	if (retries >= NUMBER_OF_RETRIES)
		return -1;

	return 0;
}

/**
 * @brief	Reports on all configured devices
 *
 * @param	detail	:	Level of detail requested
 * @return	0 on success, -1 on failure
 */
static long
report(int detail)
{
	uint32_t		i;
	struct in_addr	address;

	for (i = 0; i < deviceCount; i++)
	{
		printf("===Start of EVG Device Report===\n");
		address.s_addr	=	devices[i].ip;
		printf("Found %s @ %s:%u\n", devices[i].name, inet_ntoa(address), ntohs(devices[i].port));
	}
		printf("===End of EVG Device Report===\n\n");

	return 0;
}

/*
 * Configuration and registration functions and variables
 */
static 	const 	iocshArg		configureArg0 	= 	{ "name",		iocshArgString };
static 	const 	iocshArg		configureArg1 	= 	{ "ip",			iocshArgString };
static 	const 	iocshArg		configureArg2 	= 	{ "port",		iocshArgString };
static 	const 	iocshArg		configureArg3 	= 	{ "frequency", 	iocshArgString };
static 	const 	iocshArg*		configureArgs[] = 
{
    &configureArg0,
    &configureArg1,
    &configureArg2,
};
static	const	iocshFuncDef	configureDef	=	{ "evrConfigure", 4, configureArgs };
static 	long	configure(char *name, char *ip, char* port, char* frequency)
{

	struct sockaddr_in	address;

	if (deviceCount >= NUMBER_OF_DEVICES)
	{
		errlogPrintf("\x1B[31mUnable to configure device: Too many devices\r\n\x1B[0m");
		return -1;
	}
	if (!name || !strlen(name) || strlen(name) >= NAME_LENGTH)
	{
		errlogPrintf("\x1B[31mUnable to configure device: Missing or incorrect name\r\n\x1B[0m");
		return -1;
	}
	if (!ip || !inet_aton(ip, &address.sin_addr))
	{
		errlogPrintf("\x1B[31mUnable to configure device: Missing or incorrect ip\r\n\x1B[0m");
		return -1;
	}
	if (!port || !strlen(port) || !atoi(port) || atoi(port) > USHRT_MAX)
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect port\r\n\x1B[0m");
		return -1;
	}
	if (!frequency || !strlen(frequency) || !atoi(frequency))
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect name\r\n\x1B[0m");
		return -1;
	}

	strcpy(devices[deviceCount].name, 	name);
	devices[deviceCount].ip			=	inet_addr(ip);
	devices[deviceCount].port		=	htons(atoi(port));
	devices[deviceCount].frequency	=	atoi(frequency);

	deviceCount++;

	return 0;
}

static void configureFunc (const iocshArgBuf *args)
{
    configure(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}

static void evgRegister(void)
{
	iocshRegister(&configureDef, configureFunc);
}

/*
 * Registry export
 */
static drvet drvevg = {
    2,
    report,
    init
};
epicsExportAddress(drvet, drvevg);
epicsExportRegistrar(evgRegister);
