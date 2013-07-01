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
*fixes:
1.0b1:first raw release
1.0b2:fixed 8 bit chips
coded by Max MC Costa for s.u.m.o.t.o.y [sumotoy(at)gmail.com]
--------------------------------------------------------------------------------------
Library works with most arduino compatible micros and Teensy2/3
*/

#include <SPI.h>
#include "mcp23xxx.h"
#include "utility/_maxBitUtility.h"
//---------- constructor ----------------------------------------------------

void mcp23xxx::beginGPIO(const uint8_t cs_pin,const byte addr,uint8_t type,bool disableHInit){

 	this->_hw_addr = addr;
	this->_read_cmd  = (addr << 1) | 1;
	this->_write_cmd = addr << 1;
	this->_cs_pin = cs_pin;
	this->_mcpType = 0;
	this->_useSpi = 0;
	byte init_sequence = 0;
	switch (type){
		case MCP23S17:
			this->_mcpPorts = 16;
			this->_mcpType = type;
			this->_useSpi = 1;
			//_IOCON/_IODIR/_GPPU/_GPIO/_GPINTEN/_IPOL/_DEFVAL/_INTF/_INTCAP/_OLAT
			this->_IOCON = 0x0A;
			this->_IODIR = 0x00;
			this->_GPPU = 0x0C;
			this->_GPIO = 0x12;
			this->_GPINTEN = 0x04;
			this->_IPOL = 0x02;
			this->_DEFVAL = 0x06;
			this->_INTF = 0x0E;
			this->_INTCAP = 0x10;
			this->_OLAT = 0x14;
			this->_INTCON = 0x08;
			if (this->_hw_addr >= 0x20){
				init_sequence = 0b00101000;
			} else {
				init_sequence = 0b00100000;
			}
			this->_use8Bits = false;
		break;
		case MCP23S08:
			this->_mcpPorts = 8;
			this->_mcpType = type;
			this->_useSpi = 1;
		/*IOCON
		NC/NC/SEQOP/DISSLW/HAEN/ODR/INTPOL/NC
		*/
			this->_IOCON = 0x05;
			this->_IODIR = 0x00;
			this->_GPPU = 0x06;
			this->_GPIO = 0x09;
			this->_GPINTEN = 0x02;
			this->_IPOL = 0x01;
			this->_DEFVAL = 0x03;
			this->_INTF = 0x07;
			this->_INTCAP = 0x08;
			this->_OLAT = 0x0A;
			this->_INTCON = 0x04;
			if (this->_hw_addr >= 0){
				init_sequence = 0b00101000;
			} else {
				init_sequence = 0b00100000;
			}
			this->_use8Bits = true;
			
		break;
		case MCP23017:
			this->_mcpPorts = 16;
			this->_mcpType = type;
			this->_IOCON = 0x0A;
			this->_IODIR = 0x00;
			this->_GPPU = 0x0C;
			this->_GPIO = 0x12;
			this->_GPINTEN = 0x04;
			this->_IPOL = 0x02;
			this->_DEFVAL = 0x06;
			this->_INTF = 0x0E;
			this->_INTCAP = 0x10;
			this->_OLAT = 0x14;
			this->_INTCON = 0x04;
			init_sequence = 0b00100000;
			this->_use8Bits = false;
		break;
		case MCP2308:
			this->_mcpPorts = 8;
			this->_mcpType = type;
			this->_IOCON = 0x05;
			this->_IODIR = 0x00;
			this->_GPPU = 0x06;
			this->_GPIO = 0x09;
			this->_GPINTEN = 0x02;
			this->_IPOL = 0x01;
			this->_DEFVAL = 0x03;
			this->_INTF = 0x07;
			this->_INTCAP = 0x08;
			this->_OLAT = 0x0A;
			this->_INTCON = 0x04;
			init_sequence = 0b00100000;
			this->_use8Bits = true;
		break;
		case MCP23S18:
			this->_mcpPorts = 16;
			this->_mcpType = type;
			this->_useSpi = 1;
			this->_IOCON = 0x0A;
			this->_IODIR = 0x00;
			this->_GPPU = 0x0C;
			this->_GPIO = 0x12;
			this->_GPINTEN = 0x04;
			this->_IPOL = 0x02;
			this->_DEFVAL = 0x06;
			this->_INTF = 0x0E;
			this->_INTCAP = 0x10;
			this->_OLAT = 0x14;
			this->_INTCON = 0x08;
			if (this->_hw_addr >= 0x20){
				init_sequence = 0b00101000;
			} else {
				init_sequence = 0b00100000;
			}
			this->_use8Bits = false;
		break;
		case MCP23018:
			this->_mcpPorts = 16;
			this->_mcpType = type;
			this->_useSpi = 0;
			this->_IOCON = 0x0A;
			this->_IODIR = 0x00;
			this->_GPPU = 0x0C;
			this->_GPIO = 0x12;
			this->_GPINTEN = 0x04;
			this->_IPOL = 0x02;
			this->_DEFVAL = 0x06;
			this->_INTF = 0x0E;
			this->_INTCAP = 0x10;
			this->_OLAT = 0x14;
			this->_INTCON = 0x08;
			init_sequence = 0b00100000;
			this->_use8Bits = false;
		break;
		case MCP23016:
			this->_mcpPorts = 16;
			this->_mcpType = type;
			this->_useSpi = 0;
			this->_IOCON = 0x0A;
			this->_IODIR = 0x06;
			//this->_GPPU = 0x0C;
			this->_GPIO = 0x00;
			//this->_GPINTEN = 0x04;
			this->_IPOL = 0x04;
			//this->_DEFVAL = 0x06;
			//this->_INTF = 0x0E;
			this->_INTCAP = 0x09;
			this->_OLAT = 0x02;
			//this->_INTCON = 0x04;
			init_sequence = 0b00100000;
			this->_use8Bits = false;
		break;
		default:
			this->_mcpPorts = 0;
			this->_mcpType = 0;
			this->_use8Bits = false;
	};
	if (this->_useSpi == 1){
	//------------------------------SPI
		if (!disableHInit) {
			SPI.begin();
#if defined(__MK20DX128__)
			SPI_CLOCK_DIV4;
#elif defined(__arm__)//dunnoyet!
			SPI_CLOCK_DIV4;
#endif
			delay(100);
			::pinMode(this->_cs_pin,OUTPUT);
			::digitalWrite(this->_cs_pin,HIGH);
		}
		write_byte(this->_IOCON,init_sequence);//init sequence!!!
	} else {
	//------------------------------I2C
	}
	_gpioDirection = 0xFFFF;//all in
	_gpioState = 0x0000;//all low 
}
/*------------------ protected -----------------------------------------------
----------------------------------------------------------------------------*/

//------------------ low level -----------------------------------------------
void mcp23xxx::startSend(bool mode){
	if (this->_useSpi == 1){
#if defined(__FASTSPI)
		::digitalWriteFast(this->_cs_pin, LOW);
#else
		::digitalWrite(this->_cs_pin, LOW);
#endif
		if (mode){//IN
			SPI.transfer(this->_read_cmd);
		} else {//OUT
			SPI.transfer(this->_write_cmd);
		}
	} else {
	}
}

void mcp23xxx::endSend(){
	if (this->_useSpi == 1){
#if defined(__FASTSPI)
		::digitalWriteFast(this->_cs_pin, HIGH);
#else
		::digitalWrite(this->_cs_pin, HIGH);
#endif
	} else {
	}
}


void mcp23xxx::write_word(byte addr, word data){
	startSend(0);
	SPI.transfer(addr);
	if (!this->_use8Bits) SPI.transfer(uint16_low_byte(data));
	SPI.transfer(uint16_high_byte(data));
	endSend();
}

void mcp23xxx::write_byte(byte addr, byte data){
	startSend(0);
	SPI.transfer(addr);
	SPI.transfer(data);
	endSend();
}


word mcp23xxx::read_addr(byte addr){
	byte low_byte = 0x00;
	startSend(1);
	SPI.transfer(addr);
	if (!this->_use8Bits) low_byte  = SPI.transfer(0x0);
	byte high_byte = SPI.transfer(0x0);
	endSend();
	return byte2uint16(high_byte,low_byte);
}

//------------------ utility (author unknow) --------------------------
uint16_t mcp23xxx::byte2uint16(byte high_byte,byte low_byte){
	return (word)high_byte<<8 | (word)low_byte;
}

byte mcp23xxx::uint16_high_byte(uint16_t uint16){
	return (byte)(uint16>>8);
}

byte mcp23xxx::uint16_low_byte(uint16_t uint16){
	return (byte)(uint16 & 0x00FF);
}
/*---------- public ----------------------------------------------------
----------------------------------------------------------------------*/
//ok 8/16 bits
void mcp23xxx::gpioPinMode(bool mode){
	if (mode == INPUT){
		_gpioDirection = 0xFFFF;
	} else {
		_gpioDirection = 0x0000;
		_gpioState = 0x0000;
	}
	write_word(this->_IODIR,_gpioDirection);
}

//ok 8/16 bits
void mcp23xxx::gpioPort(word value){
	if (this->_use8Bits) {
		_gpioState = value<<8;
	} else {
		_gpioState = value;
	}
	write_word(this->_GPIO,_gpioState);
}


void mcp23xxx::gpioPort(byte lowByte, byte highByte){
	_gpioState = byte2uint16(highByte,lowByte);
	write_word(this->_GPIO,_gpioState);
}

//should ok for 8/16 bits
uint16_t mcp23xxx::gpioPort(){
	return read_addr(this->_GPIO);
}

//should ok for 8/16 bits
uint16_t mcp23xxx::gpioPortFast(){
	if (this->_use8Bits) {
		return _gpioState<<8;
	} else {
		return _gpioState;
	}
}

void mcp23xxx::gpioPinMode(uint8_t pin, bool mode){
	if (pin < _mcpPorts){
		if (this->_use8Bits) pin = pin+8;
		if (mode == INPUT){
			BIT_SET(_gpioDirection,pin);
		} else {
			BIT_CLEAR(_gpioDirection,pin);
		}
		write_word(this->_IODIR,_gpioDirection);
	}
}

//fast!
void mcp23xxx::gpioDigitalWrite(uint8_t pin, bool value){
	if (pin < _mcpPorts){
		if (this->_use8Bits) pin = pin+8;
		if (value){
			BIT_SET(_gpioState,pin);
		} else {
			BIT_CLEAR(_gpioState,pin);
		}
		write_word(this->_GPIO,_gpioState);
	}
}

int mcp23xxx::gpioDigitalRead(uint8_t pin){
	if (pin < _mcpPorts) (int)(read_addr(this->_GPIO) & 1<<pin);
}

int mcp23xxx::gpioDigitalReadFast(uint8_t pin){
	if (pin < _mcpPorts){
		if (this->_use8Bits) pin = pin+8;
		int temp = BIT_CHECK(_gpioState,pin);
		return temp;
	} else {
		return 0;
	}
}

/* void mcp23xxx::printBits(word data,uint8_t bits){
  for (int i=(bits-1); i>=0; i--){
    if (bitRead(data,i)==1){
      Serial.print("1");
    } 
    else {
      Serial.print("0");
    }
  }
  Serial.print(" -> 0x");
  Serial.print(data,HEX);
  Serial.print("\n");
}
 */