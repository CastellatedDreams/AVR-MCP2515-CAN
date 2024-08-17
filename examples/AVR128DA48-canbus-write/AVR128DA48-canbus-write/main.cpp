/*
 * AVR128DA48-canbus-write.cpp
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



//Variables
//____________________________________________________________________________________________________
uint16_t counter = 256;		//dummy value for incrementing 



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
	
	message.id = 0x00AA;
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
	//increment the counter
	counter++;
	
	//example modification of can bus message. Here you may update the data with a sensor reading
	message.id = 0x00AB;
	message.len = 2;
	message.data[0] = (counter & 0xff00) >> 8;
	message.data[1] = (counter & 0x00ff);
	
	//send message
	can.sendMessage(&message);
	
	//delay 100ms before sending the next message
	_delay_ms(100);
	PORTC.OUTTGL |= PIN6_bm;
}



//Start
//____________________________________________________________________________________________________
int main(void)
{
	setup();
		
    while (1) 
		loop();

}

