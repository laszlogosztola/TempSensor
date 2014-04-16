/*
 * RFMTestProject.c
 *
 * Created: 2014.02.05. 15:53:41
 *  Author: Ghost
 */ 


#include "config.h"
#include "serial.h"

#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "include/rfm12_config.h"
#include "include/rfm12_hw.h"
#include "include/rfm12_core.h"

void set_led_state(uint8_t led, uint8_t state);
uint16_t rfm12_read(uint16_t c);
void rfm12_data(uint16_t data);
void rfSend(uint8_t data);

#define SCK 7 //SPI clock
#define SDO 6 //SPI data out
#define SDI 5 //SPI data in
#define CS	4 //chip select
#define NIRQ 2

#define HI(x) PORTB |= (1<<(x))
#define LO(x) PORTB &= ~(1<<(x))
#define WAIT_NIRQ_LOW() while(PIND&(1<<NIRQ))


void rfm12_data(uint16_t data)
{
	rfm12_read(data);
}

uint16_t rfm12_read(uint16_t data)
{
	uint16_t recv = 0;
	LO(SCK);
	LO(CS);
	for(uint8_t i=0; i<16; i++)
	{
		if ( data & 0x8000 )
		HI(SDI);
		else
		LO(SDI);
		
		HI(SCK);
		recv<<=1;
		if (PINB & (1<<SDO) )
		{
			recv|=0x0001;
		}
		LO(SCK);
		data<<=1;
	}
	HI(CS);
	return recv;
}


void init_rfm(void)
{
	//setup selected band
	rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);
	/*
	//Init device
	//enable internal data register and fifo
	
	rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | 0x07);
	
	volatile uint16_t s = 0;
	//send_status_via_usb(s);
	//set power default state (usually disable clock output)
	//do not write the power register two times in a short time
	//as it seems to need some recovery
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
	
	
	rfm12_data(0xA640);
	rfm12_data(0xc647);
	
	rfm12_data(0x94a0);
	rfm12_data(0xc2ac);
	rfm12_data(0xca81);
	
	rfm12_data(0xced4);
	
	rfm12_data(0xc483);
	rfm12_data(0x9850);
	rfm12_data(0xcc17);
	rfm12_data(0xe000);
	rfm12_data(0xc800);
	rfm12_data(0xc040);
	
	rfm12_read(0);
	*/
	
	rfm12_data(0x8239); //!er,!ebb,ET,ES,EX,!eb,!ew,DC
	rfm12_data(0xA640); //frequency select
	rfm12_data(0xC647); //4.8kbps
	rfm12_data(0x94A0); //VDI,FAST,134kHz,0dBm,-103dBm
	rfm12_data(0xC2AC); //AL,!ml,DIG,DQD4
	rfm12_data(0xCA81); //FIFO8,SYNC,!ff,DR
	rfm12_data(0xCED4); //SYNC=2DD4 , AG
	rfm12_data(0xC483); //@PWR,NO RSTRIC,!st,!fi,OE,EN
	rfm12_data(0x9850); //!mp,90kHz,MAX OUT
	rfm12_data(0xCC17); //OB1 , ACOB0, LPX,Iddy,CDDIT,CBW0
	rfm12_data(0xE000); //NOT USED
	rfm12_data(0xC800); //NOT USED
	rfm12_data(0xC040); //1.66MHz,2.2V
/*
	//set frequency
	//rfm12_data(RFM12_CMD_FREQUENCY | RFM12_FREQUENCY_CALC(FREQ) );
	rfm12_data(RFM12_CMD_FREQUENCY | 0x680);

	//set data rate
	rfm12_data(RFM12_CMD_DATARATE | DATARATE_VALUE );
	
	
	//set rx parameters: int-in/vdi-out pin is vdi-out,
	//Bandwith, LNA, RSSI
	//rfm12_data(RFM12_CMD_RXCTRL | RFM12_RXCTRL_P16_VDI | RFM12_RXCTRL_VDI_FAST | RFM12_RXCTRL_BW_400 | RFM12_RXCTRL_LNA_6 | RFM12_RXCTRL_RSSI_79 );
	rfm12_data(RFM12_CMD_RXCTRL | RFM12_RXCTRL_VDI_FAST | RFM12_RXCTRL_BW_400 | RFM12_RXCTRL_LNA_6 | RFM12_RXCTRL_RSSI_79 );
	
	//automatic clock lock control(AL), digital Filter(!S),
	//Data quality detector value 3, slow clock recovery lock
	rfm12_data(RFM12_CMD_DATAFILTER | RFM12_DATAFILTER_AL | 3);
	
	//2 Byte Sync Pattern, Start fifo fill when sychron pattern received,
	//disable sensitive reset, Fifo filled interrupt at 8 bits
	//rfm12_data(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4));
	rfm12_data(RFM12_CMD_FIFORESET | RFM12_FIFORESET_AL | RFM12_FIFORESET_DR | (8<<4));

	//set AFC to automatic, (+4 or -3)*2.5kHz Limit, fine mode, active and enabled
	rfm12_data(RFM12_CMD_AFC | RFM12_AFC_AUTO_KEEP | RFM12_AFC_LIMIT_4 | RFM12_AFC_FI | RFM12_AFC_OE | RFM12_AFC_EN);
	
	//set TX Power to -0dB, frequency shift = +-125kHz
	rfm12_data(RFM12_CMD_TXCONF | RFM12_TXCONF_POWER_0 | RFM12_TXCONF_FS_CALC(125000) );
	
	//disable low dutycycle mode
	rfm12_data(RFM12_CMD_DUTYCYCLE);
	
	//disable wakeup timer
	rfm12_data(RFM12_CMD_WAKEUP);
	
	//enable rf receiver chain, if receiving is not disabled (default)
	//the magic is done via defines
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
*/		
}

void rfSend(uint8_t data)
{
	WAIT_NIRQ_LOW();
	rfm12_data(0xB800 | data);
}

#define DELAYTIME	0

void send_data(void)
{
	while ( 1 )
	{
		rfm12_read(0);

		rfSend(0xAA);
		_delay_ms(DELAYTIME);
		rfSend(0xAA);
		_delay_ms(DELAYTIME);
		rfSend(0xAA);
		_delay_ms(DELAYTIME);
		rfSend(0x2D);
		_delay_ms(DELAYTIME);
		rfSend(0xD4);
		_delay_ms(DELAYTIME);
	
		rfSend('0');
		_delay_ms(DELAYTIME);
		rfSend('1');
		_delay_ms(DELAYTIME);
		rfSend('2');
		_delay_ms(DELAYTIME);
		rfSend('3');
		_delay_ms(DELAYTIME);
	
		rfSend(0xC6);
		_delay_ms(DELAYTIME);
		rfSend(0xAA);
		_delay_ms(DELAYTIME);
	
		_delay_ms(1000);
	}
	
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
}

static volatile uint16_t data;
static volatile uint16_t data2;

void receive(void)
{

	volatile uint8_t readed[100];
	volatile uint16_t count = 0;
	volatile uint16_t dummy = 0;
	//rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);
	
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | PWRMGT_RECEIVE);
	
	/*rfm12_data(0x8299); //er,!ebb,ET,ES,EX,!eb,!ew,DC (bug was here)
	rfm12_data(0xA640); //freq select
	rfm12_data(0xC647); //4.8kbps
	//rfm12_data(0x94A0); //VDI,FAST,134kHz,0dBm,-103dBm
	rfm12_data(0x90A0); //no VDI,FAST,134kHz,0dBm,-103dBm
	rfm12_data(0xC2AC); //AL,!ml,DIG,DQD4
	rfm12_data(0xCA81); //FIFO8,SYNC,!ff,DR (FIFO level = 8)
	rfm12_data(0xCED4); //SYNC=2DD4;
	rfm12_data(0xC483); //@PWR,NO RSTRIC,!st,!fi,OE,EN
	rfm12_data(0x9850); //!mp,90kHz,MAX OUT
	rfm12_data(0xCC17); //!OB1,!OB0, LPX,!ddy,DDIT,BW0
	rfm12_data(0xE000); //NOT USE
	rfm12_data(0xC800); //NOT USE
	rfm12_data(0xC040); //1.66MHz,2.2V
	*/
	
	
	_delay_us(100);
	rfm12_read(0);
	
	while ( 1 )
	{
		//reset FIFO
		rfm12_data(0xCA81);
		rfm12_data(0xCA83);
		
		count = 0;
		
		for ( int i = 0; i < 3; i++ )
		{
			WAIT_NIRQ_LOW();
			data = rfm12_read(0);
			if ( data & 0xfc00 )
			{
				dummy++;
			}
			if ( data & 0x8000 )
			{
				data2 = rfm12_read(0xB000);
				readed[count++] = (uint8_t)(data2 & 0xff);
			}	
		}
		
	}
}


int main(void)
{
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	wdt_disable();
	
	sei();
	
	//set direction registers ( 1 - output, 0 - input )
	DDRB = (1<<CS) | (1<<SDI) | (1<<SCK) | (1<<LED1_PIN) | ( 1<<LED2_PIN);
	DDRD = 0;
	
	HI(CS);
	HI(SDI);
	LO(SCK);
	
	
	init_serial_port(9600);
	
	for ( int i = 0; i < 3; i++ )
	{
		set_led_state(2, 1);
		_delay_ms(300);
		set_led_state(2, 0);
		_delay_ms(350);
	}
	
	init_rfm();
	//receive();
	
    while(1)
    {
        set_led_state(1, 1);
        _delay_ms(300);
        set_led_state(1, 0);
        _delay_ms(350);
		send_data();
    }
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