/*
 * AVR_MCP2515_CANBUS.h
 *
 * This library provides an interface to control and manage the MCP2515 CAN Bus controller using AVR microcontrollers and 
 * facilitates communication over the CAN bus.
 * The code base is designed to be as simple as possible to ensure ease of use, and its setup and usage reflect the familiar 
 * Arduino Serial communication patterns with functions such as .begin(), .sendMessage(), .available(), and .readMessage().
 * 
 *
 *
 * Supported Microcontrollers:
 *		- AVR128DA28
 *		- AVR128DA32
 *		- AVR128DA48
 *		- AVR128DA64
 *		- ATtiny202
 *		- ATtiny404
 *		- ATtiny424
 *
 * Limitations:
 *		- The library only supports the use of the SPI0 peripheral. If the use of SPI1 or other SPI ports is strictly necessary,
 *		  please update the library accordingly.
 *		- The library currently supports only standard CAN bus frames. It does not support extended data frames, remote frames,
 *		  active error frames, or overload frames.
 *		- The library only supports MCP2515 devices run by a 8MHz crystal, if a 16MHz or 20MHz crystal is used, please update the 
 *		  library accordingly
 *
 * Usage:
 *		- Add both the AVR-MCP2515-CANBUS.h and AVR-MCP2515-CANBUS.cpp in your project directory
 *		- Include the header file in your main.cpp (#include "AVR-MCP2515-CANBUS.h")
 * 		- See the example can-bus-write.cpp and can-bus-read.cpp for how to use the can bus functions
 *
 * Updated: 2024-08-17
 * Author: Castellated Dreams - Rein Åsmund Torsvik 
 */



#ifndef AVR-MCP2515-CANBUS_H_
#define AVR-MCP2515-CANBUS_H_

// set SPI pin configuration based on chip
//____________________________________________________________________________________________________
#if defined(__AVR_AVR128DA28__) || defined(__AVR_AVR128DA32__) || defined(__AVR_AVR128DA48__) || defined(__AVR_AVR128DA64__)
	#define SPI_PORT	&PORTA
	#define PIN_MOSI	PIN4_bm
	#define PIN_MISO	PIN5_bm
	#define PIN_SCK		PIN6_bm
	#define PIN_nSS		PIN7_bm
#elif defined(__AVR_ATtiny202__)
	#define SPI_PORT	&PORTA
	#define PIN_MOSI	PIN1_bm
	#define PIN_MISO	PIN2_bm
	#define PIN_SCK		PIN3_bm
	#define PIN_nSS		PIN0_bm
#elif defined(__AVR_ATtiny404__) || defined(__AVR_ATtiny424__)
	#define SPI_PORT	&PORTA
	#define PIN_MOSI	PIN1_bm
	#define PIN_MISO	PIN2_bm
	#define PIN_SCK		PIN3_bm
	#define PIN_nSS		PIN4_bm
#else
	#error "This microcontroller is unsupported by the AVR-MCP2515-CANBUS library. Please update this library with the pinouts of your controller"
#endif



// MCP2515 setup parameters
//____________________________________________________________________________________________________
enum CAN_BITRATE_t {
	CAN_50kbps,
	CAN_83k3bps,
	CAN_95kbps,
	CAN_100kbps,
	CAN_125kbps,
	CAN_200kbps,
	CAN_250kbps,
	CAN_500kbps,
	CAN_1000kbps
};

enum MCP2515_MODE_t {
	MCP2515_MODE_CONFIG,
	MCP2515_MODE_NORMAL,
	MCP2515_MODE_SLEEP,
	MCP2515_MODE_LISTENONLY,
	MCP2515_MODE_LOOPBACK
};



// CAN message struct
//____________________________________________________________________________________________________
typedef struct {
	uint16_t id;
	uint8_t len;
	uint8_t data[8];
} can_msg;



// MCP2515 SPI instructions and registers
//____________________________________________________________________________________________________
#define	MCP2515_INSTRUCTION_RESET	0b11000000
#define	MCP2515_INSTRUCTION_READ	0b00000011
#define	MCP2515_INSTRUCTION_WRITE	0b00000010
#define	MCP2515_INSTRUCTION_READRX0	0b10010000
#define	MCP2515_INSTRUCTION_READRX1	0b10010100
#define MCP2515_INSTRUCTION_LOADTX0	0b01000000
#define	MCP2515_INSTRUCTION_LOADTX1 0b01000010
#define	MCP2515_INSTRUCTION_LOADTX2	0b01000100
#define	MCP2515_INSTRUCTION_RTSTX0	0b10000001
#define	MCP2515_INSTRUCTION_RTSTX1	0b10000010
#define	MCP2515_INSTRUCTION_RTSTX2	0b10000100
#define	MCP2515_INSTRUCTION_GETSTAT	0b10100000
#define	MCP2515_INSTRUCTION_RXSTAT	0b10110000
#define	MCP2515_INSTRUCTION_MODBIT	0b00000101 

#define MCP2515_REGISTER_CNF1		0x2A
#define MCP2515_REGISTER_CNF2		0x29
#define MCP2515_REGISTER_CNF3		0x28
#define MCP2515_REGISTER_CANCTRL	0x0F
#define MCP2515_REGISTER_CANSTAT	0x0E

#define MCP2515_STATUS_RX0IF_bm		0x01
#define MCP2515_STATUS_RX1IF_bm		0x02
#define MCP2515_STATUS_TX0REQ_bm	0x04
#define MCP2515_STATUS_TX0IF_bm		0x08
#define MCP2515_STATUS_TX1REQ_bm	0x10
#define MCP2515_STATUS_TX1IF_bm		0x20
#define MCP2515_STATUS_TX2REQ_bm	0x40
#define MCP2515_STATUS_TX2IF_bm		0x80

#define CANCTRL_REQOP_NORMAL_gc		0b000 << 5
#define CANCTRL_REQOP_SLEEP_gc		0b001 << 5
#define CANCTRL_REQOP_LOOPBACK_gc	0b010 << 5
#define CANCTRL_REQOP_LISTENONLY_gc 0b011 << 5
#define CANCTRL_REQOP_CONFIG_gc		0b100 << 5
#define CANCTRL_OSM_bm				0x08

#define MCP2515_REGISTER_TXB0CTRL	0x30
#define MCP2515_REGISTER_TXB0SIDH	0x31
#define MCP2515_REGISTER_TXB0SIDL	0x32
#define MCP2515_REGISTER_TXB0EID8	0x33
#define MCP2515_REGISTER_TXB0EID0	0x34
#define MCP2515_REGISTER_TXB0DLC	0x35
#define MCP2515_REGISTER_TXB0D0		0x36
#define MCP2515_REGISTER_TXB0D1		0x37
#define MCP2515_REGISTER_TXB0D2		0x38
#define MCP2515_REGISTER_TXB0D3		0x39
#define MCP2515_REGISTER_TXB0D4		0x3A
#define MCP2515_REGISTER_TXB0D5		0x3B
#define MCP2515_REGISTER_TXB0D6		0x3C
#define MCP2515_REGISTER_TXB0D7		0x3D

#define MCP2515_REGISTER_TXB1CTRL	0x40
#define MCP2515_REGISTER_TXB2CTRL	0x50


#define TXBnCTRL_ABTF_bm			0x40
#define TXBnCTRL_MLOA_bm			0x20
#define TXBnCTRL_TXERR_bm			0x10
#define TXBnCTRL_TXREQ_bm			0x08
#define TXBnSIDL_EXID_bm			0x08
#define TXBnDLC_RTR_bm				0x40
#define TXBnDLC_DLC_gc				0b00001111


#define RXBnSIDL_SID_gc				0b11100000
#define RXBnSIDL_SRR_bm				0x10
#define RXBnSIDL_IDE_bm				0x08
#define RXBnDLC_DLC_gc				0b00001111





class MCP2515_CAN
{
	public:
		
		bool begin(CAN_BITRATE_t can_speed);
		void reset();				//reset the MCP2515 device. This puts the device in config mode and bitrate must be set by set_bitrate() before be returning the device to normal mode using set_mode(MCP2515_MODE_NORMAL)
		
		uint8_t sendMessage(can_msg *msg);
		
		uint8_t available();		//check if any messages are recieved in the recieve buffers, returns 0, 1 or 2
		can_msg readMessage();		//read CAN message
		
		MCP2515_MODE_t setMode(MCP2515_MODE_t mode);
		void setBitrate(CAN_BITRATE_t bitrate);
		
	
	private:
	
		void _spi_init(PORT_t *port);
		void _spi_open();
		void _spi_close();
		uint8_t _spi_write(uint8_t data);
		
		void _mcp_reset();
		void _mpc_write(uint8_t register_address, uint8_t data);
		uint8_t _mcp_read(uint8_t register_address);
		
		void _mcp_loadtxbn(uint8_t n, can_msg *msg);		//load TX buffer n
		void _mcp_rtsn(uint8_t n);							//request to sent buffer n
		
		uint8_t _mcp_read_status();								//read the MCP status
		
		uint8_t* _mcp_readrxbn(uint8_t n);
		uint8_t _mcp_rxstatus();							//get the RX status using the RX status instruction
		
		
		
	
		
	
		
};



#endif /* CD-AVR128DA48-MCP2515-CANBUS_H_ */