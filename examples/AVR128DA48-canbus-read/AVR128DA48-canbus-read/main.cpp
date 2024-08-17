/*
 * AVR128DA48-canbus-read.cpp
 *
 * Created: 26.07.2024 13:48:57
 * Author : reint
 */ 

#define F_CPU 16000000UL // 16 MHz



//Inclusions
//____________________________________________________________________________________________________
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "AVR-MCP2515-CANBUS.h"



//Pin definitions
//____________________________________________________________________________________________________
#define LED_PIN PIN6_bm		//AVR128DA48 Curiosity nano board LED pin



//Objects
//____________________________________________________________________________________________________
MCP2515_CAN can;
can_msg message;



//Setup
//____________________________________________________________________________________________________
void setup()
{
	//Clock configurations
	CCP = CCP_IOREG_gc;								//unlock protected register
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;	// Select high-frequency internal oscillator
	
	CCP = CCP_IOREG_gc; 
	CLKCTRL.MCLKCTRLB &= ~CLKCTRL_PEN_bm;			// disable clock division
		
	CCP = CCP_IOREG_gc; 
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FRQSEL_16M_gc;		// Set clock frequency to 16 MHz
	
	
	
	// PIN configuration and setup
	PORTC.DIRSET = LED_PIN;
	PORTC.OUT |= PIN6_bm;
	
	
	
	//CAN bus setup
	can.begin(CAN_125kbps);
	
	message.id = 0x00BE;
	message.len = 8;
	message.data[0] = 0x01;
	message.data[1] = 0x02;
	message.data[2] = 0x03;
	message.data[3] = 0x04;
	message.data[4] = 0x05;
	message.data[5] = 0x06;
	message.data[6] = 0x07;
	message.data[7] = 0x08;
	
	
	
	//setup complete
	PORTC.OUT &= ~PIN6_bm;
	
}



//Main loop
//____________________________________________________________________________________________________
void loop()
{
	
	//Check if there are any messages available in the MCP2515 buffer
	if(can.available() > 0)
	{
		//read the message
		message = can.readMessage();
		
		//toggle the LED if a message was recieved from a node with address 0x00AB
		if(message.id == 0x00AB)
			PORTC.OUTTGL |= PIN6_bm;
		
	}

	_delay_ms(100);

}



//Start
//____________________________________________________________________________________________________
int main(void)
{
	setup();
		
    while (1) 
		loop();

}

