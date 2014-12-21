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
	REGISTER_SEQ_CLOCK_SEL	=	0x24,
	REGISTER_AC_ENABLE		=	0x28,
	REGISTER_MXC_CONTROL	=	0x2A,
	REGISTER_MXC_PRESCALER	=	0x2C,
	REGISTER_RF_CONTROL		=	0x40,
	REGISTER_SEQ_ADDRESS	=	0x44,
	REGISTER_SEQ_CODE		=	0x46,
	REGISTER_SEQ_TIME		=	0x48,
	REGISTER_USEC_DIVIDER	=	0x68,
} evgregister_t;

/*Register definitions*/
#define CONTROL_DISABLE			0x8000
#define CONTROL_FIFO_FULL		0x4000
#define CONTROL_DISABLE_FIFO	0x1000
#define CONTROL_ERROR_LED		0x0800
#define CONTROL_RX_VIOLATION	0x0001
#define EVENT_ENABLE_SEQUENCE	0x0004
#define AC_ENABLE_SYNC			0x4000
#define AC_ENABLE_DIVIDER		50
#define MXC_CONTROL_HIGH_WORD	0x0008
#define RF_CONTROL_EXTERNAL		0x01C0
#define RF_CONTROL_DIVIDER		0x0003
#define USEC_DIVIDER			125

#define EVENT_END_SEQUENCE		0x7f

/*evg UDP packet field defitions*/
#define ACCESS_READ		(1)
#define ACCESS_WRITE	(2)

/*Device name maximum length*/
#define NAME_LENGTH				30
/*evg register base address*/
#define REGISTER_BASE_ADDRESS	0x80000000

#define NUMBER_OF_EVENTS	100

/** 
 * @brief	Searches for device with given name and returns a pointer to it 
 * @return	Void pointer to found device, NULL otherwise
 */
void*	evg_open					(char *name);
/**
 * @brief	enables/disables the device
 *
 * Enables/disables the device. Always leaves the upstream receiver disabled.
 *
 * @param	*dev	:	a pointer to the device being acted upon
 * @param	enable	:	enables the device if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evg_enable					(void* device, bool enable);
/**
 * @brief	Enables/disables the sequencer
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the sequencer if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evg_enableSequencer			(void* device, bool enable);
/**
 * @brief	Enables/disables the AC trigger
 *
 * Enables/disables the AC trigger. Leaves the rest of the enable register intact.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the AC trigger if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evg_enableAcTrigger			(void* device, bool enable);
/**
 * @brief	Sets the RF clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of RF's clock
 * @return	0 on success, -1 on failure
 */
long	evg_setRfPrescaler			(void* device, uint8_t prescaler);
/**
 * @brief	Sets the sequencer clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of the sequencer's clock
 * @return	0 on success, -1 on failure
 */
long	evg_setSequencerPrescaler	(void* device, uint16_t prescaler);
/**
 * @brief	Sets the AC trigger clock's prescaler
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	prescaler	:	The prescaler (divisor) of the AC trigger's clock
 * @return	0 on success, -1 on failure
 */
long	evg_setAcTriggerPrescaler	(void* device, uint8_t prescaler);
/**
 * @brief	Sets an event code at the specified address
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	addrsess	:	The address of the event
 * @param	event		:	The event
 * @return	0 on success, -1 on failure
 */
long	evg_setEvent				(void* device, uint16_t address, uint8_t event);
/**
 * @brief	Sets a timestamp at the specified address
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	addrsess	:	The address of the event
 * @param	event		:	The timestamp
 * @return	0 on success, -1 on failure
 */
long	evg_setTimestamp			(void* device, uint16_t address, uint32_t timestamp);
/**
 * @brief	Sets the prescaler of the counter's clock
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	counter		:	The counter being operated on
 * @param	prescaler	:	The prescaler (divisor) of the counter's clocke
 * @return	0 on success, -1 on failure
 */
long	evg_setCounterPrescaler		(void* device, uint8_t counter, uint32_t prescaler);

#endif /*__EVG_H__*/
