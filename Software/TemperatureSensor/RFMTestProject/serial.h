/*
 * serial.h
 *
 * Created: 2014.02.02. 11:11:24
 *  Author: Ghost
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

#include "config.h"

#include <inttypes.h>

void init_serial_port(uint16_t baud);

void send_byte_array_synchron(uint8_t* data, uint8_t length);
void send_string_synchron(char* data);

uint8_t is_ready_for_send(void);



#endif /* SERIAL_H_ */