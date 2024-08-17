/*
 * CAN-bus-write.cpp
 *
 * Created: 26.07.2024 13:48:57
 * Author : reint
 */ 

#define F_CPU 16000000UL // 16 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "AVR-MCP2515-CANBUS.h"



#define LED_PIN PIN6_bm

MCP2515_CAN can;
can_msg message;
can_msg response;




void setup()
{
	//Clock configurations
	CCP = CCP_IOREG_gc; //unlock protected register
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc; // Select high-frequency internal oscillator
	
	CCP = CCP_IOREG_gc; //unlock protected register
	CLKCTRL.MCLKCTRLB &= ~CLKCTRL_PEN_bm; // disable clock division
		
	CCP = CCP_IOREG_gc; //unlock protected register
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FRQSEL_16M_gc; // Set clock frequency to 16 MHz
	
	
	
	// PIN configuration and setup
	PORTC.DIRSET = LED_PIN;
	PORTC.OUT |= PIN6_bm;
	
	
	
	//CAN bus setup
	can.begin(CAN_125KBPS);
	
	message.id = 0x00FE;
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

void loop()
{
	// Toggle PC6
	_delay_ms(100);
	PORTC.OUTTGL |= PIN6_bm;
	
	
	
	
// 	uint8_t isavailable = can.available();
// 	if(isavailable)
// 	{
// 		response = can.readMessage();
// 		
// 		if(response.id == 0x00FE)
// 		PORTC.OUTTGL |= PIN6_bm;
// 		
// 	}
	
	message.data[7] = ++counter;
	can.sendMessage(&message);
	
	

	
}


int main(void)
{
	setup();
		
    /* Replace with your application code */
    while (1) 
    {
		loop();
    }
}

