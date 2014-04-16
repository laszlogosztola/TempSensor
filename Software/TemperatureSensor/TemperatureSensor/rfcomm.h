/*
 * rfcomm.h
 *
 * Created: 2014.02.07. 19:39:16
 *  Author: Ghost
 */ 


#ifndef RFCOMM_H_
#define RFCOMM_H_

#include "config.h"

void init_rf_module(void);
void send_byte_package(uint8_t* data, uint8_t length);


#endif /* RFCOMM_H_ */