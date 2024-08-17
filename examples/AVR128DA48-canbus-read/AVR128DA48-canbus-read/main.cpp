/*
 * AVR128DA48-canbus-read.cpp
 *
 * This program demonstrates the use of the AVR-MCP2515-CANBUS library to read messages from the can bus
 * The program initializes an AVR128DA48 microcontroller and configures the MCP2515 to communicate at 125kbps. 
 * It then continuously checks for incoming CAN messages and toggles an on-board LED if a message from a 
 * specific CAN ID (0x00AB) is received.
 *
 * Updated: 2024-08-17
 * Author: Castellated Dreams - Rein Åsmund Torsvik
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

}



//Start
//____________________________________________________________________________________________________
int main(void)
{
	setup();
		
    while (1) 
		loop();

}

