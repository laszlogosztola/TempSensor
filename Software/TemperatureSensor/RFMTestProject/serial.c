/*
 * serial.c
 *
 * Created: 2014.02.02. 11:12:22
 *  Author: Ghost
 */ 

#include "serial.h"

static volatile uint8_t _ready_for_send = 0;

void init_serial_port(uint16_t baud)
{
	UCSRB = (1 << RXEN) | ( 1 << TXEN );
	UCSRC = ( 1 << UCSZ0 ) | ( 1 <<UCSZ1 );
	uint16_t temp = ((uint32_t)F_CPU / ((uint32_t)16 * baud))-1;
	UBRRH = (temp >> 8) & 0xff;
	UBRRL = temp & 0xff;
	_ready_for_send = 1;
}

void send_byte_array_synchron(uint8_t* data, uint8_t length)
{
	_ready_for_send = 0;
	uint8_t sent_bytes = 0;
	while ( sent_bytes < length )
	{
		while (( UCSRA & ( 1 << UDRE )) == 0) { }
		UDR = data[sent_bytes];
		sent_bytes++;
	}
	_ready_for_send = 1;
}

void send_string_synchron(char* data)
{
	_ready_for_send = 0;
	char* send_ptr = data;
	while ( *send_ptr != 0 )
	{
		while (( UCSRA & ( 1 << UDRE )) == 0) { }
		UDR = *send_ptr;
		send_ptr++;		
	}
	_ready_for_send = 1;
}

uint8_t is_ready_for_send(void)
{
	return _ready_for_send;	
}