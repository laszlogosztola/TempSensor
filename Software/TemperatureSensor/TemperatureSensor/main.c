/*
 * main.c
 *
 * Created: 2013.12.17. 5:05:41
 *  Author: Ghost
 */ 

/************************************************************************/
/* Pin definitions                                                      */
/*																		*/
/* PB3: debug LED 1														*/	
/* PB2: debug LED 2														*/
/* PB0: supply for AM2302												*/
/* PB1: communication for AM2302										*/
/* ...																	*/
/*																		*/
/************************************************************************/
#include "config.h"
#include "am2302.h"
#include "serial.h"
#include "rfcomm.h"

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

void set_led_state(uint8_t led, uint8_t state);
void read_temperature_data(void);

//Watchdog timeout ISR
ISR(WDT_OVERFLOW_vect)
{
	wdt_reset();
	//set up WDT interrupt (has to set WDIE after each interrupt)
	_WD_CONTROL_REG = (1<<WDCE) | (1<<WDE);
	_WD_CONTROL_REG = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (1<<WDP0);

	set_led_state(2, 1);
	_delay_ms(200);
	set_led_state(2, 0);
	//reti();
}

int main(void)
{
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	wdt_disable();
	
	init_serial_port(9600);
	init_rf_module();
	
	//TODO: set-up unused pins to avoid floating!
	
	//set direction registers ( 1 - output, 0 - input )
	DDRB |= ( 1 << LED1_PIN ) | ( 1 << LED2_PIN );
	
	set_led_state(1, 0);
	set_led_state(2, 0);
	
	
	//set up WDT interrupt
	_WD_CONTROL_REG = (1<<WDCE) | (1<<WDE);
	//Start watchdog timer with 8s prescaler
	_WD_CONTROL_REG = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (1<<WDP0);
	
	//Enable global interrupts
	sei();
	
	for ( int i = 0; i < 3; i++ )
	{
		set_led_state(1, 1);
		_delay_ms(100);
		set_led_state(1, 0);
		_delay_ms(350);
	}
	
	uint8_t cycle_count = 0;
	
	while ( 1 )
	{
		cycle_count++;
		if ( cycle_count == 3 )
		{
			for ( int i = 0; i < 2; i++ )
			{
				set_led_state(2, 1);
				_delay_ms(200);
				set_led_state(2, 0);
				_delay_ms(200);
			}
			wdt_reset();
			read_temperature_data();
			cycle_count = 0;
		}else
		{
			set_led_state(1, 1);
			_delay_ms(1000);
			set_led_state(1, 0);	
		}
		
		//wdt_reset();
		
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sei();
		sleep_mode();

	}
	
	/*
	float temperature, humidity;
	while(1)
    {
        _delay_ms(1000);
		power_up_sensor();
		_delay_ms(2500);
		temperature = 0.0;
		humidity = 0.0;
		read_data(&temperature, &humidity);
		
		_delay_ms(1500);
		power_down_sensor();
		
    }*/
}

void read_temperature_data(void)
{
	float temperature, humidity;
	
	power_up_sensor();
	_delay_ms(2200);
	temperature = 0.0;
	humidity = 0.0;
	read_data(&temperature, &humidity);
	
	_delay_ms(100);
	power_down_sensor();
}

void set_led_state(uint8_t led, uint8_t state)
{
	if ( state == 1 )
	{
		if ( led == 1 )
		{
			LED1_PORT |= ( 1 << LED1_PIN );
		}else if ( led == 2 )
		{
			LED2_PORT |= ( 1 << LED2_PIN );
		}
	}else
	{
		if ( led == 1 )
		{
			LED1_PORT &= ~(1 << LED1_PIN );
		}else if ( led == 2 )
		{
			LED2_PORT &= ~(1 << LED2_PIN );
		}
	}
}