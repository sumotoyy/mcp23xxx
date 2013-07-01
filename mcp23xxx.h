/*
                               _                
 ___  _   _  _ __ ___    ___  | |_  ___   _   _ 
/ __|| | | || '_ ` _ \  / _ \ | __|/ _ \ | | | |
\__ \| |_| || | | | | || (_) || |_| (_) || |_| |
|___/ \__,_||_| |_| |_| \___/  \__|\___/  \__, |
                                          |___/ 
MCP23xxx I2C/SPI basic Library
--------------------------------------------------------------------------------------
A simple C library for SPI/I2C GPIO Expander chip MCP23xxx (I2C/SPI)
that can be easily included in other libraries. It support the best
features of every chip (like extended addressing in SPI that support 8
chips by using the same CS wire).
version 1.0b2
coded by Max MC Costa for s.u.m.o.t.o.y [sumotoy(at)gmail.com]
*fixes:
1.0b1:first raw release
1.0b2:fixed 8 bit chips
--------------------------------------------------------------------------------------
Library works with most arduino compatible micros and Teensy2/3
*/
#ifndef mcp23xxx_h
#define mcp23xxx_h

#if ARDUINO < 100
	#include "WProgram.h"
#else
	#include "Arduino.h"
#endif

#if defined(__arm__) // Arduino Due

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__)

#elif defined(__MK20DX128__)   // for Teensy 3.0
#define __FASTSPI
#else
//#define __FASTSPI
#endif

/*
address:  0 0100 A2 A1 A0
			MCP23S17
				__ __
		IOB-0 [|  U  |] IOA-7
		IOB-1 [|     |] IOA-6
		IOB-2 [|     |] IOA-5
		IOB-3 [|     |] IOA-4
		IOB-4 [|     |] IOA-3
		IOB-5 [|     |] IOA-2
		IOB-6 [|     |] IOA-1
		IOB-7 [|     |] IOA-0
		++++  [|     |] INT-A
		GND   [|     |] INT-B
		CS    [|     |] RST
		SCK   [|     |] A2
		MOSI  [|     |] A1
		MISO  [|_____|] A0
---------------------------------------
address:  0 0100 A2 A1 A0
			MCP23017
				__ __
		IOB-0 [|  U  |] IOA-7
		IOB-1 [|     |] IOA-6
		IOB-2 [|     |] IOA-5
		IOB-3 [|     |] IOA-4
		IOB-4 [|     |] IOA-3
		IOB-5 [|     |] IOA-2
		IOB-6 [|     |] IOA-1
		IOB-7 [|     |] IOA-0
		++++  [|     |] INT-A
		GND   [|     |] INT-B
		-NC-  [|     |] RST
		SCL   [|     |] A2
		SDA   [|     |] A1
		-NC-  [|_____|] A0	
*/
/* #define IOCONA   0x0A
#define IOCON    0x0A 
#define IODIRA   0x00
#define IODIRB   0x01
#define IODIR    0x00

#define GPPUA    0x0C
#define GPPUB    0x0D
#define GPPU     0x0C

#define GPIOA    0x12
#define GPIOB    0x13
#define GPIO     0x12 */

    typedef enum MCP_type{
        MCP23S17 = 1,
		MCP23S08 = 2,
        MCP23017 = 3,
		MCP2308  = 4,
		MCP23S18 = 5,
		MCP23018 = 6,
		MCP23016 = 7,
		MCP2309 = 8
    };
		
class mcp23xxx
{
  public:

	void 			beginGPIO(const uint8_t cs_pin,const byte addr,uint8_t type,bool disableHInit=false);
    virtual void 	gpioPinMode(bool mode);
	virtual void 	gpioPinMode(uint8_t pin, bool mode);
    virtual void 	gpioPort(word value);
	virtual void 	gpioPort(byte lowByte, byte highByte);
    uint16_t 		gpioPort();
	uint16_t 		gpioPortFast();
    
    virtual void 	gpioDigitalWrite(uint8_t pin, bool value);
    virtual int  	gpioDigitalRead(uint8_t pin);
	virtual int  	gpioDigitalReadFast(uint8_t pin);
	/* void 			printBits(word data,uint8_t bits); */

  protected:

    uint8_t 		_cs_pin;
    byte 			_hw_addr;
    byte 			_read_cmd;
    byte 			_write_cmd;
	byte 			_mcpType;
	byte			_mcpPorts;
	byte			_useSpi;
	word			_gpioDirection;
	word			_gpioState;

    word 			read_addr(byte addr);
    void 			write_word(byte addr, word data);
	void 			write_byte(byte addr, byte data);
	void 			startSend(bool mode);
	void 			endSend();

    uint16_t 		byte2uint16(byte high_byte, byte low_byte);
    byte 			uint16_high_byte(uint16_t uint16);
    byte 			uint16_low_byte(uint16_t uint16);
	
	
	byte			_IOCON;
	byte			_IODIR;
	byte			_GPPU;
	byte			_GPIO;
	
	byte			_GPINTEN;
	byte			_IPOL;
	byte			_DEFVAL;
	byte			_INTF;
	byte			_INTCAP;
	byte			_OLAT;
	byte			_INTCON;
	bool			_use8Bits;
};

#endif 
