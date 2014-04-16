/*
 * rfcomm.c
 *
 * Created: 2014.02.07. 19:39:28
 *  Author: Ghost
 */ 

#include "rfcomm.h"


#include "include/rfm12_config.h"
#include "include/rfm12_hw.h"
#include "include/rfm12_core.h"


//send 2 bytes to the RF module
void rfm12_data(uint16_t data);

//send 2 bytes to the RF module and read answer
uint16_t rfm12_read(uint16_t data);

void rfSend(uint8_t data);



void init_rf_module(void)
{
	//setup selected band
	rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);
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
}

#define DELAYTIME 0

void send_byte_package(uint8_t* data, uint8_t length)
{
	rfm12_read(0);

	//TODO: do we need to send 0xAA two times?
	rfSend(0xAA);
	_delay_ms(DELAYTIME);
	rfSend(0xAA);
	_delay_ms(DELAYTIME);
	
	rfSend(0x2D);
	_delay_ms(DELAYTIME);
	rfSend(0xD4);
	_delay_ms(DELAYTIME);
	
	for ( int i = 0; i < length; i++ )
	{
		rfSend(data[i]);
		_delay_ms(DELAYTIME);
	}
	
	//deactivate TX
	//TODO: check if DEFAULT is ok for me? wakeup, etc
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
}



/************************************************************************/
/*                                                                      */
/************************************************************************/

//TODO: modify to use hardware SPI!
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

void rfSend(uint8_t data)
{
	WAIT_NIRQ_LOW();
	rfm12_data(0xB800 | data);
}
