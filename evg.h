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
 * @file 	evg.h
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	2014-10-09
 * @brief	Header file for the epics driver support layer for the VME-evg-230/RF timing card
 */

#ifndef __EVG_H__
#define __EVG_H__

/*System headers*/
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief	VME-MRF-230/RF Register addresses
 */
typedef enum
{
	REGISTER_CONTROL		=	0x00,
	REGISTER_EVENT_ENABLE	=	0x02,
	REGISTER_SW_EVENT		=	0x04,
	REGISTER_SEQ_CLOCK_SEL1	=	0x24,
	REGISTER_SEQ_CLOCK_SEL2	=	0x26,
	REGISTER_AC_ENABLE		=	0x28,
	REGISTER_MXC_CONTROL	=	0x2A,
	REGISTER_MXC_PRESCALER	=	0x2C,
	REGISTER_FIRMWARE		=	0x2E,
	REGISTER_RF_CONTROL		=	0x40,
	REGISTER_SEQ_ADDRESS0	=	0x44,
	REGISTER_SEQ_CODE0		=	0x46,
	REGISTER_SEQ_TIME0		=	0x48,
	REGISTER_SEQ_ADDRESS1	=	0x50,
	REGISTER_SEQ_CODE1		=	0x52,
	REGISTER_SEQ_TIME1		=	0x54,
	REGISTER_USEC_DIVIDER	=	0x68,
} evgregister_t;

/*Register definitions*/
#define CONTROL_DISABLE			0xF001
#define CONTROL_DISABLE_BIT		0x8000
#define CONTROL_ENABLE			0x7001
#define CONTROL_VTRG1			0x0100
#define CONTROL_VTRG2			0x0080
#define EVENT_ENABLE_VME		0x0001
#define EVENT_ENABLE_SEQUENCER1	0x0002
#define EVENT_ENABLE_SEQUENCER0	0x0004
#define AC_ENABLE_SEQ1			0x8000
#define AC_ENABLE_SEQ0			0x4000
#define AC_ENABLE_SYNC			0x1000
#define AC_ENABLE_DIVIDER_MASK	0x00FF
#define MXC_CONTROL_HIGH_WORD	0x0008
#define RF_CONTROL_EXTERNAL		0x01C0
#define RF_CONTROL_DIVIDER_MASK	0x003F
#define USEC_DIVIDER			125

#define EVENT_END_SEQUENCE		0x7f

/*evg UDP packet field defitions*/
#define ACCESS_READ		(1)
#define ACCESS_WRITE	(2)

/*Device name maximum length*/
#define NAME_LENGTH				30
/*evg register base address*/
#define REGISTER_BASE_ADDRESS	0x80000000

#define NUMBER_OF_EVENTS		100
#define NUMBER_OF_SEQUENCERS	2
#define NUMBER_OF_ADDRESSES		2048
#define NUMBER_OF_COUNTERS		8
#define MAX_EVENT_FREQUENCY		125

typedef enum
{
	RF_SOURCE_INTERNAL,
	RF_SOURCE_EXTERNAL
} rfsource_t;

typedef enum
{
	AC_SOURCE_EVENT,
	AC_SOURCE_MXC7
} acsource_t;

typedef enum
{
	TRIGGER_SOFT,
	TRIGGER_AC
} triggersource_t;

void*	evg_open						(char *name);
long	evg_enable						(void* device, bool enable);
long	evg_isEnabled					(void* device);
long	evg_setClock					(void* device, uint16_t frequency);
long	evg_getClock					(void* device, uint16_t *frequency);
long	evg_setRfClockSource			(void* device, rfsource_t source);
long	evg_getRfClockSource			(void* device, rfsource_t *source);
long	evg_setRfPrescaler				(void* device, uint8_t prescaler);
long	evg_getRfPrescaler				(void* device, uint8_t *prescaler);
long	evg_setAcPrescaler				(void* device, uint8_t prescaler);
long	evg_getAcPrescaler				(void* device, uint8_t *prescaler);
long	evg_setAcSyncSource				(void* device, acsource_t source);
long	evg_getAcSyncSource				(void* device, acsource_t *source);
long	evg_enableSequencer				(void* device, uint8_t sequencer, bool enable);
long	evg_isSequencerEnabled			(void* device, uint8_t sequencer);
long	evg_setSequencerTriggerSource	(void* device, uint8_t sequencer, triggersource_t source);
long	evg_getSequencerTriggerSource	(void* device, uint8_t sequencer, triggersource_t *source);
long	evg_setSequencerPrescaler		(void* device, uint8_t sequencer, uint16_t prescaler);
long	evg_getSequencerPrescaler		(void* device, uint8_t sequencer, uint16_t *prescaler);
long	evg_triggerSequencer			(void* device, uint8_t sequencer);
long	evg_setEvent					(void* device, uint8_t sequencer, uint16_t address, uint8_t event);
long	evg_getEvent					(void* device, uint8_t sequencer, uint16_t address, uint8_t *event);
long	evg_setTimestamp				(void* device, uint8_t sequencer, uint16_t address, uint32_t timestamp);
long	evg_getTimestamp				(void* device, uint8_t sequencer, uint16_t address, uint32_t *timestamp);
long	evg_setSoftwareEvent			(void* device, uint8_t event);
long	evg_setCounterPrescaler			(void* device, uint8_t counter, uint32_t prescaler);
long	evg_getCounterPrescaler			(void* device, uint8_t counter, uint32_t *prescaler);
long	evg_getFirmwareVersion			(void* device, uint16_t *version);

#endif /*__EVG_H__*/
