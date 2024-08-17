/*
 * CD_AVR128DA48_MCP2515_CANBUS.cpp
 *
 * Created: 26.07.2024 14:21:18
 *  Author: reint
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "AVR-MCP2515-CANBUS.h"


//SPI functions
//____________________________________________________________________________________________________
void MCP2515_CAN::_spi_init(PORT_t *port)
{
	//configure pins
	port->DIRSET	= PIN_MOSI 
					| PIN_SCK 
					| PIN_nSS; 
					
	port->OUT		|= PIN_nSS;			//set SS pin to 1 (Slave Select off)
					
	port->DIRCLR	= PIN_MISO;
	
	//configure SPI peripheral
	SPI0.CTRLA		= SPI_MASTER_bm			//set device as master
					| SPI_PRESC_DIV4_gc		//set clock prescalar to 4
					| SPI_CLK2X_bm;			//conficure double clock speed
					
	SPI0.CTRLB		= SPI_MODE_0_gc;		//set clock syncronisation mode
	
	//enable SPI peripheral
	SPI0.CTRLA		|= SPI_ENABLE_bm;
}

void MCP2515_CAN::_spi_open()
{
	PORTA.OUT &= ~PIN7_bm;
	return;
}

void MCP2515_CAN::_spi_close()
{
	PORTA.OUT |= PIN7_bm;
	return;
}

uint8_t MCP2515_CAN::_spi_write(uint8_t data)
{
	SPI0.DATA = data;						// Send data
	while (!(SPI0.INTFLAGS & SPI_IF_bm));	// Wait for data register to be empty
	
	uint8_t response =  SPI0.DATA;			// extract the response shifted into SPI0.DATA
	return response;
}



//MCP2515 functions
//____________________________________________________________________________________________________
void MCP2515_CAN::_mcp_reset()
{
	_spi_open();
	_spi_write(MCP2515_INSTRUCTION_RESET);
	_spi_close();
	return;
}

uint8_t MCP2515_CAN::_mcp_read_status()
{
	_spi_open();
	_spi_write(MCP2515_INSTRUCTION_GETSTAT);
	
	uint8_t status = _spi_write(0);
	_spi_close();
	
	return status;
}

void MCP2515_CAN::_mpc_write(uint8_t register_address, uint8_t data)
{
	_spi_open();
	_spi_write(MCP2515_INSTRUCTION_WRITE);
	_spi_write(register_address);
	_spi_write(data);
	_spi_close();	
}

uint8_t MCP2515_CAN::_mcp_read(uint8_t register_address)
{
	_spi_open();
	_spi_write(MCP2515_INSTRUCTION_READ);
	_spi_write(register_address);
	uint8_t data = _spi_write(0);
	_spi_close();
	
	return data;
}

void MCP2515_CAN::_mcp_loadtxbn(uint8_t n, can_msg *msg)
{
	//construct 13 byte frame containing ID, length and can message payload (data) to parse to the MCP2515 tx buffer. 
	//This frame is not a CAN bus frame, just information to the MCP2515 buffer
	//see MCP2515 data sheet page 15 to 22 (register 3-1 to 3-8) 
	
	uint8_t frame[13];
	
	frame[0] = (msg->id >> 3) & 0xFF;;	//shift the 11bit ID down 3 places and store in the first byte
	frame[1] = (msg->id << 5) & 0xE0;	//shift the 3 last bits of the ID up. Add TXBnSIDL_EXID_mb to enable extended CAN ID
	frame[2] = 0;						//set extended IDh to 0 (not in use)
	frame[3] = 0;						//set extended IDl to 0 (not in use)
	frame[4] = msg->len & 0x0F;			//set data length. Add TXBnDLC_RTR to enable Remote Transmision Request
	frame[5] = msg->data[0];			//set data bytes
	frame[6] = msg->data[1];
	frame[7] = msg->data[2];
	frame[8] = msg->data[3];
	frame[9] = msg->data[4];
	frame[10] = msg->data[5];
	frame[11] = msg->data[6];
	frame[12] = msg->data[7];
	
	//parse frame
	_spi_open();
	
	if (n == 2)
		_spi_write(MCP2515_INSTRUCTION_LOADTX2);
	else if (n == 1)
		_spi_write(MCP2515_INSTRUCTION_LOADTX1);
	else
		_spi_write(MCP2515_INSTRUCTION_LOADTX0);
		
	for (uint8_t i = 0; i < 13; i++)
	{
		_spi_write(frame[i]);
	}
	
	_spi_close();
}

void MCP2515_CAN::_mcp_rtsn(uint8_t n)
{
	_spi_open();
	if (n == 2)
		_spi_write(MCP2515_INSTRUCTION_RTSTX2);
	else if (n == 1)
		_spi_write(MCP2515_INSTRUCTION_RTSTX1);
	else
		_spi_write(MCP2515_INSTRUCTION_RTSTX0);
	_spi_close();
}

uint8_t MCP2515_CAN::_mcp_rxstatus()
{
	_spi_open();
	_spi_write(MCP2515_INSTRUCTION_RXSTAT);
	uint8_t rxstatus = _spi_write(0);
	_spi_close();
	return rxstatus;	
}

uint8_t* MCP2515_CAN::_mcp_readrxbn(uint8_t n)
{
	//send read RXBn command
	_spi_open();
	if (n == 1)
		_spi_write(MCP2515_INSTRUCTION_READRX1);
	else
		_spi_write(MCP2515_INSTRUCTION_READRX0);
		
	//read RX buffer n
	static uint8_t rxbn[13]; 
	
	 for (int i = 0; i < 13; i++) {
		 rxbn[i] = _spi_write(0);
	 }
	
	return rxbn;
}



//MCP2515 initialization and setup functions. Functions for sending and receiving messages
//____________________________________________________________________________________________________
bool MCP2515_CAN::begin(CAN_BITRATE_t can_speed)
{
	//initialise SPI peripheral
	_spi_init(SPI_PORT);
	
	//send dummy message to ensure first transmission which sets the INTFLAGS
	_spi_open();
	_spi_write(0);
	_spi_close();
	
	//reset the MCP2515 device
	_mcp_reset();
	
	//initialize the MCP2515
	//MCP2515 starts in configuration mode. The CNF1, CNF2, CNF3, filters and masks are only configurable in this mode.
	setBitrate(can_speed);
	
	//start the MCP2515 by setting it in normal mode and return true if mode is set successfully
	MCP2515_MODE_t mode = setMode(MCP2515_MODE_NORMAL);
	if (mode == MCP2515_MODE_NORMAL)
		return true;
	
	return false;
}

void MCP2515_CAN::reset()
{
	_mcp_reset();
}

uint8_t  MCP2515_CAN::sendMessage(can_msg *msg)
{
	//Check if message is ok
	//TODO	
	
	//Check if any if the three TX buffers are free. If not, return false
	uint8_t n = -1;
	if (!(_mcp_read(MCP2515_REGISTER_TXB0CTRL) & TXBnCTRL_TXREQ_bm))
		n = 0;
	else if (!(_mcp_read(MCP2515_REGISTER_TXB1CTRL) & TXBnCTRL_TXREQ_bm))
		n = 1;
	else if (!(_mcp_read(MCP2515_REGISTER_TXB2CTRL) & TXBnCTRL_TXREQ_bm))
		n = 2;
	else
		return false;
	
	//load message into TXBn buffer and request to send
	_mcp_loadtxbn(n, msg);
	_mcp_rtsn(n);
	return n;
}

uint8_t MCP2515_CAN::available()
{
	uint8_t rxstatus = _mcp_rxstatus();
	uint8_t n = (rxstatus >> 6) & 0x03;
	return n;
}

can_msg MCP2515_CAN::readMessage()
{
	can_msg msg;
	uint8_t* rxbn;
	
	//check which buffer is full an ready to read
	uint8_t status = _mcp_read_status();
	if (status & MCP2515_STATUS_TX0IF_bm)
		rxbn = _mcp_readrxbn(0);
	
	else if (status & MCP2515_STATUS_TX1IF_bm)
		rxbn = _mcp_readrxbn(1);
		
	else
		return msg;
	
	//extract the ID
	msg.id = (rxbn[0] << 3) | ((rxbn[1] & RXBnSIDL_SID_gc) >> 5);

	//extract data length
	msg.len = rxbn[4] & RXBnDLC_DLC_gc;  // DLC is the lower 4 bits of frame[4]

	// Extract data
	for (int i = 0; i < msg.len; i++) 
	{
		msg.data[i] = rxbn[5 + i];
	}

	return msg;
}

MCP2515_MODE_t MCP2515_CAN::setMode(MCP2515_MODE_t mode)
{
	//see MCP2515 Documentation page 60, REGISTER 10-1
	uint8_t configuration;
	switch (mode)
	{
		case MCP2515_MODE_CONFIG:
		configuration = CANCTRL_REQOP_CONFIG_gc | CANCTRL_OSM_bm; //CANCTRL REQOP to 000, and CANCTRL OCM to 1 (one shot mode, can bus will attempt to send a message only once)
		break;
		
		case MCP2515_MODE_NORMAL:
		configuration = CANCTRL_REQOP_NORMAL_gc | CANCTRL_OSM_bm;
		break;
		
		case MCP2515_MODE_SLEEP:
		configuration = CANCTRL_REQOP_SLEEP_gc | CANCTRL_OSM_bm;
		break;
		
		case MCP2515_MODE_LISTENONLY:
		configuration = CANCTRL_REQOP_LISTENONLY_gc | CANCTRL_OSM_bm;
		break;
		
		case MCP2515_MODE_LOOPBACK:
		configuration = CANCTRL_REQOP_LOOPBACK_gc | CANCTRL_OSM_bm;
		break;
		
		default:
		// Handle invalid mode case
		break;
	}
	
	//write configuration
	_mpc_write(MCP2515_REGISTER_CANCTRL, configuration);
	_spi_write(0);
	
	//check can status register to confirm mode
	uint8_t data = _mcp_read(MCP2515_REGISTER_CANSTAT);
	
	switch (data & 0b11100000)
	{
		case CANCTRL_REQOP_NORMAL_gc:
		return MCP2515_MODE_NORMAL;
		
		case CANCTRL_REQOP_SLEEP_gc:
		return MCP2515_MODE_SLEEP;
		
		case CANCTRL_REQOP_LOOPBACK_gc:
		return MCP2515_MODE_LOOPBACK;
		
		case CANCTRL_REQOP_LISTENONLY_gc:
		return MCP2515_MODE_LISTENONLY;
		
		case CANCTRL_REQOP_CONFIG_gc:
		return MCP2515_MODE_CONFIG;
		
		default:
		return static_cast<MCP2515_MODE_t>(-1);
	}
	
}

void MCP2515_CAN::setBitrate(CAN_BITRATE_t bitrate)
{
	uint8_t cnf1, cnf2, cnf3;
	
	switch (bitrate)
	{
		case CAN_50kbps:
		cnf1 = 0x03;
		cnf2 = 0xB4;
		cnf3 = 0x86;
		break;
		
		case CAN_83k3bps:
		cnf1 = 0x47;
		cnf2 = 0xE2;
		cnf3 = 0x85;
		break;
		
		case CAN_95kbps:
		cnf1 = 0x01;
		cnf2 = 0xB4;
		cnf3 = 0x86;
		break;
		
		case CAN_100kbps:
		cnf1 = 0x01;
		cnf2 = 0xB4;
		cnf3 = 0x86;
		break;
		
		case CAN_125kbps:
		cnf1 = 0x01;
		cnf2 = 0xB1;
		cnf3 = 0x85;
		break;
		
		case CAN_200kbps:
		cnf1 = 0x00;
		cnf2 = 0xB4;
		cnf3 = 0x86;
		break;
		
		case CAN_250kbps:
		cnf1 = 0x00;
		cnf2 = 0xB1;
		cnf3 = 0x85;
		break;
		
		case CAN_500kbps:
		cnf1 = 0x00;
		cnf2 = 0x90;
		cnf3 = 0x82;
		break;
		
		case CAN_1000kbps:
		cnf1 = 0x00;
		cnf2 = 0x80;
		cnf3 = 0x80;
		break;
		
		default:
		// Handle invalid bitrate case
		cnf1 = 0xFF;
		cnf2 = 0xFF;
		cnf3 = 0xFF;
		break;
	}
	
	_mpc_write(MCP2515_REGISTER_CNF1, cnf1);
	_mpc_write(MCP2515_REGISTER_CNF2, cnf2);
	_mpc_write(MCP2515_REGISTER_CNF3, cnf3);
	
}


