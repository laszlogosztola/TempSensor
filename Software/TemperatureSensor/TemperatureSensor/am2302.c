/*
 * am2302.c
 *
 * Created: 2013.12.17. 5:37:06
 *  Author: Ghost
 */ 

#include "am2302.h"
#include "serial.h"

#include <avr/interrupt.h>

static volatile uint8_t pin_change_count = 0;
static volatile uint16_t last_edge_point = 0;

#define MAX_NUM_EDGES  45
static uint8_t edges[MAX_NUM_EDGES];

static uint8_t readed_data[6];

/*
To decode sensor data, should start a timer and set-up an IRQ on falling edge of the comm. pin.
If the current bit has 0 value, bit width is ~80usec, if bit value is 1, bit width is ~120 usec
SDA pin of AM2302 is connected to PB1/pin13/PCINT1 of the MCU
*/

ISR(PCINT_B_vect)
{
	//falling edge on comm pin and not too much edge
	if ((( AM2302_COMM_PORT_INP & ( 1 << AM2302_COMM_PIN ) ) == 0 ) && ( pin_change_count < MAX_NUM_EDGES ))
	{
		uint16_t temp = TCNT1;
		//from 3rd falling edge we have data
		if ( pin_change_count > 1 )
		{
			edges[pin_change_count-2] = temp-last_edge_point;
			if (( temp - last_edge_point ) > 100)
			{
				readed_data[(pin_change_count-2) / 8] |= 1 << (7 - (pin_change_count-2) % 8);
			}
			
		}
		last_edge_point = temp;
		pin_change_count++;	
	}
}

void power_up_sensor(void)
{
	//set communication and supply pin as output
	AM2302_COMM_DIR |= ( 1 << AM2302_COMM_PIN );
	AM2302_SUPP_DIR |= ( 1 << AM2302_SUPP_PIN );
	
	//set the comm pin to high to be able to pull low later
	AM2302_COMM_PORT |= ( 1 << AM2302_COMM_PIN );
	
	//set the pin to high -> power up the sensor
	AM2302_SUPP_PORT |= ( 1 << AM2302_SUPP_PIN );
}

void power_down_sensor(void)
{
	//set the pin to high -> power up the sensor
	AM2302_SUPP_PORT &= ~( 1 << AM2302_SUPP_PIN );
}

//Read data - returns 1 if successful read happened
uint8_t read_data(float* temperature, float* humidity)
{
	pin_change_count = 0;
	last_edge_point = 0;
	for ( int i = 0; i < 40; i++ )
	{
		edges[i] = 0;
	}
	for ( int i = 0; i < 6; i++ )
	{
		readed_data[i] = 0;
	}
	
	//Set up timer
	TCNT1 = 0x0000;
	TCCR1A = 0x00;
	//prescaler: 8
	TCCR1B |= (1 << CS11);
	
	//trigger start
	//pull the output low
	AM2302_COMM_PORT &= ~( 1 << AM2302_COMM_PIN );
	_delay_ms(1);
	
	//set up IRQ - store the timestamp of falling edges
	PCMSK2 = 0x00;
	PCMSK1 = 0x00;
	//enable PCINT1
	PCMSK = 0x02;
	//Bit 5 is the PCIE0 irq bit
	GIMSK |= 1 << 5;

	sei();
	
	//and change the pin to input
	AM2302_COMM_DIR &= ~( 1 << AM2302_COMM_PIN );
	
	//and wait for data
	//TODO: find out why delay is strange here (
	_delay_ms(10);
	
	PCMSK = 0x00;
	
	uint8_t sum = readed_data[0] + readed_data[1] + readed_data[2] + readed_data[3];
	if (( sum != readed_data[4] ) || ( sum == 0 ))
	{
		*temperature = 101.0;
		*humidity = 101.0;
	}else
	{
		uint16_t humidity_temp = (((uint16_t)readed_data[0]) << 8) | readed_data[1];
		uint16_t temperature_temp = (((uint16_t)readed_data[2]) << 8) | readed_data[3];
		*humidity = (float)humidity_temp / 10.0;
		*temperature = (float)temperature_temp / 10.0;	
	}
	
	send_string_synchron("Finish ");
	//send_byte_array_synchron(&pin_change_count, 1);
	//send_string_synchron(" ");
	//send_byte_array_synchron(edges, 40);
	//send_string_synchron(" ");
	send_byte_array_synchron(readed_data, 6);
			
	return 0;
}