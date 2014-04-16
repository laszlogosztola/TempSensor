/*
 * config.h
 *
 * Created: 2013.12.17. 5:38:55
 *  Author: Ghost
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU		8000000UL

#include <avr/io.h>
#include <util/delay.h>


#define LED1_PORT	PORTB
#define LED1_PIN	3

#define LED2_PORT	PORTB
#define LED2_PIN	2

#define AM2302_SUPP_PORT	PORTB
#define AM2302_SUPP_DIR		DDRB
#define AM2302_SUPP_PIN		0

#define AM2302_COMM_PORT		PORTB
#define AM2302_COMM_PORT_INP	PINB
#define AM2302_COMM_DIR			DDRB
#define AM2302_COMM_PIN			1

#endif /* CONFIG_H_ */