/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3.0 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) SESAME (sesame.org.jo), 2014
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
static	long	init(void);
/*Reports on all configured devices*/
static	long	report(int detail);
/*Writes data to register*/
static	long	evg_write(void *dev, evgregister_t reg, uint16_t data);
/*Reads data from register*/
static	long	evg_read(void *dev, evgregister_t reg, uint16_t *data);

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
	uint32_t			i;
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

		/*Disable EVG and receiver*/
		status	=	evg_enable(&devices[device], 0);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Disable sequencer and set its prescaler to default value*/
		status	=	evg_enableSequencer(&devices[device], 0);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}
		status	=	evg_setSequencerPrescaler(&devices[device], 1);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Disable AC trigger and set its prescaler to default value*/
		status	=	evg_enableAcTrigger(&devices[device], 0);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}
		status	=	evg_setAcTriggerPrescaler(&devices[device], 50);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Initialize RF clock*/
		status	=	evg_setRfPrescaler(&devices[device], 4);
		if (status < 0)
		{
			errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Initialize events*/
		for (i = 0; i < NUMBER_OF_EVENTS; i++)
		{
			status	=	evg_setEvent(&devices[device], i, EVENT_END_SEQUENCE);
			if (status < 0)
			{
				errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
				return -1;
			}
			status	=	evg_setTimestamp(&devices[device], i, 0);
			if (status < 0)
			{
				errlogPrintf("\x1B[31mUnable to initialize device\n\x1B[0m");
				return -1;
			}
		}
	}
	return 0;
}

/**
 * @brief	enables/disables the device
 *
 * Enables/disables the device. Always leaves the upstream receiver disabled.
 *
 * @param	*dev	:	a pointer to the device being acted upon
 * @param	enable	:	enables the device if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long
evg_enable(void* dev, bool enable)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Act*/
	if (enable)
	{
		status	=	evg_write(device, REGISTER_CONTROL, CONTROL_DISABLE_FIFO);
		if (status < 0)
		{
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		/*Check*/
		status	=	evg_read(device, REGISTER_CONTROL, &data);
		if (status < 0)
		{ 
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		if (data&CONTROL_DISABLE)
		{
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	evg_write(device, REGISTER_CONTROL, CONTROL_DISABLE | CONTROL_DISABLE_FIFO);
		if (status < 0)
		{
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		/*Check*/
		status	=	evg_read(device, REGISTER_CONTROL, &data);
		if (status < 0)
		{ 
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		if (!(data&CONTROL_DISABLE))
		{
			errlogPrintf("\x1B[31menable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Enables/disables the sequencer
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the sequencer if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long
evg_enableSequencer(void* dev, bool enable)
{
	uint32_t	i;
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	if (enable)
	{
		for (i = 0; i < 5 && !(data&EVENT_ENABLE_SEQUENCE); i++)
		{
			status	=	evg_write(device, REGISTER_EVENT_ENABLE, EVENT_ENABLE_SEQUENCE);
			status	=	evg_read(device, REGISTER_EVENT_ENABLE, &data);
		}
	}
	else
	{
		for (i = 0; i < 5 && (data&EVENT_ENABLE_SEQUENCE); i++)
		{
			status	=	evg_write(device, REGISTER_EVENT_ENABLE, 0);
			status	=	evg_read(device, REGISTER_EVENT_ENABLE, &data);
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets the sequencer clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of the sequencer's clock
 * @return	0 on success, -1 on failure
 */
long
evg_setSequencerPrescaler(void* dev, uint16_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Set event frequency*/
	status	=	evg_write(device, REGISTER_SEQ_CLOCK_SEL, prescaler);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check*/
	status	=	evg_read(device, REGISTER_SEQ_CLOCK_SEL, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != prescaler)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Enables/disables the AC trigger
 *
 * Enables/disables the AC trigger. Leaves the rest of the enable register intact.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the AC trigger if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long
evg_enableAcTrigger(void* dev, bool enable)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Read original value of register*/
	status		=	evg_read(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	data	&=	~AC_ENABLE_SYNC;

	/*Enable/disable the device*/
	if (enable)
	{
		/*Act*/
		status	=	evg_write(device, REGISTER_AC_ENABLE, data|AC_ENABLE_SYNC);
		if (status < 0)
		{
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		/*Check*/
		status	=	evg_read(device, REGISTER_AC_ENABLE, &data);
		if (status < 0)
		{ 
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		if (!(data&AC_ENABLE_SYNC))
		{
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		/*Act*/
		status	=	evg_write(device, REGISTER_AC_ENABLE, data);
		if (status < 0)
		{
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		/*Check*/
		status	=	evg_read(device, REGISTER_AC_ENABLE, &data);
		if (status < 0)
		{ 
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
		if (data&AC_ENABLE_SYNC)
		{
			errlogPrintf("\x1B[31menableAcTrigger is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets the AC trigger clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of the AC trigger's clock
 * @return	0 on success, -1 on failure
 */
long
evg_setAcTriggerPrescaler(void* dev, uint8_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Read original value of register*/
	status		=	evg_read(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetAcTriggerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Act*/
	status	=	evg_write(device, REGISTER_AC_ENABLE, (data&0xff00)|prescaler);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetAcPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	/*Check*/
	status	=	evg_read(device, REGISTER_AC_ENABLE, &data);
	if (status < 0)
	{ 
		errlogPrintf("\x1B[31msetAcPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if ((uint8_t)data != prescaler)
	{
		errlogPrintf("\x1B[31msetAcPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets the RF clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of RF's clock
 * @return	0 on success, -1 on failure
 */
long
evg_setRfPrescaler(void* dev, uint8_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Set and enable RF clock*/
	status	=	evg_write(device, REGISTER_RF_CONTROL, RF_CONTROL_EXTERNAL | (prescaler-1));
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check*/
	status	=	evg_read(device, REGISTER_RF_CONTROL, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != (RF_CONTROL_EXTERNAL | (prescaler-1)))
	{
		errlogPrintf("\x1B[31msetClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets an event code at the specified address
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	addrsess	:	The address of the event
 * @param	event		:	The event
 * @return	0 on success, -1 on failure
 */
long
evg_setEvent(void* dev, uint16_t address, uint8_t event)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Set address*/
	status	=	evg_write(device, REGISTER_SEQ_ADDRESS, address);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check*/
	status	=	evg_read(device, REGISTER_SEQ_ADDRESS, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != address)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Set event*/
	status	=	evg_write(device, REGISTER_SEQ_CODE, event);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check*/
	status	=	evg_read(device, REGISTER_SEQ_CODE, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != event)
	{
		errlogPrintf("\x1B[31msetEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets a timestamp at the specified address
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	addrsess	:	The address of the event
 * @param	event		:	The timestamp
 * @return	0 on success, -1 on failure
 */
long
evg_setTimestamp(void* dev, uint16_t address, uint32_t timestamp)
{
	uint16_t	data	=	0;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check delay*/
	if (timestamp < 0 || timestamp > (UINT_MAX/USEC_DIVIDER))
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful: timestamp is too long\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert timestamp*/
	cycles	=	timestamp*USEC_DIVIDER;	

	/*Set address*/
	status	=	evg_write(device, REGISTER_SEQ_ADDRESS, address);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check*/
	status	=	evg_read(device, REGISTER_SEQ_ADDRESS, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != address)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new timestamp*/
	status	=	evg_write(device, REGISTER_SEQ_TIME, cycles>>16);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetTimestampe is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_write(device, REGISTER_SEQ_TIME+2, cycles);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check that new timestamp was updated*/
	status	=	evg_read(device, REGISTER_SEQ_TIME, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != (cycles>>16))
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_read(device, REGISTER_SEQ_TIME+2, &data);
	if (data != (uint16_t)cycles)
	{
		errlogPrintf("\x1B[31msetTimestamp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets the prescaler of the counter's clock
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	counter		:	The counter being operated on
 * @param	prescaler	:	The prescaler (divisor) of the counter's clocke
 * @return	0 on success, -1 on failure
 */
long
evg_setCounterPrescaler(void* dev, uint8_t counter, uint32_t prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check inputs*/
	if (counter > 7)
	{
		errlogPrintf("\x1B[31msetCounterPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select counter and high word*/
	status	=	evg_write(device, REGISTER_MXC_CONTROL, MXC_CONTROL_HIGH_WORD | counter);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_read(device, REGISTER_MXC_CONTROL, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != (MXC_CONTROL_HIGH_WORD | counter))
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write and check high word of prescaler*/
	status	=	evg_write(device, REGISTER_MXC_PRESCALER, prescaler>>16);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_read(device, REGISTER_MXC_PRESCALER, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != (prescaler>>16))
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select counter and low word*/
	status	=	evg_write(device, REGISTER_MXC_CONTROL, counter);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_read(device, REGISTER_MXC_CONTROL, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != counter)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write and check low word*/
	status	=	evg_write(device, REGISTER_MXC_PRESCALER, prescaler);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	evg_read(device, REGISTER_MXC_PRESCALER, &data);
	if (status < 0)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != prescaler)
	{
		errlogPrintf("\x1B[31msetSequencerPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

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
evg_read(void *dev, evgregister_t reg, uint16_t *data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

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
	{
		errlogPrintf("\x1B[31mRead is unsuccessful\n\x1B[0m");
		return -1;
	}

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
evg_write(void *dev, evgregister_t reg, uint16_t data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

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
	{
		errlogPrintf("\x1B[31mWrite is unsuccessful\n\x1B[0m");
		return -1;
	}

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
static 	const 	iocshArg		configureArg0 	= 	{ "name", 	iocshArgString };
static 	const 	iocshArg		configureArg1 	= 	{ "ip", 	iocshArgString };
static 	const 	iocshArg		configureArg2 	= 	{ "port", 	iocshArgString };
static 	const 	iocshArg*		configureArgs[] = 
{
    &configureArg0,
    &configureArg1,
    &configureArg2,
};
static	const	iocshFuncDef	configureDef	=	{ "evgConfigure", 3, configureArgs };
static 	long	configure(char *name, char *ip, char* port)
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
		errlogPrintf("\x1B[31mUnable to configure device: Missing or incorrect port\r\n\x1B[0m");
		return -1;
	}

	strcpy(devices[deviceCount].name, 	name);
	devices[deviceCount].ip		=	inet_addr(ip);
	devices[deviceCount].port	=	htons(atoi(port));

	deviceCount++;

	return 0;
}

static void configureFunc (const iocshArgBuf *args)
{
    configure(args[0].sval, args[1].sval, args[2].sval);
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
