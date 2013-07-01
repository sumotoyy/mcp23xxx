/*
2 chip example. It uses 2 MCP23S17 in parallel and thanks HAEN
featured by this chip it's possible drive 8 chips connected in the same
CS pin. Using different CS pin let you drive 8 chip per CS
(16 I/O channel * 8) * CS pin
*/

#include <SPI.h>
#include <mcp23xxx.h>

#define MCP_CSPIN  10 

mcp23xxx mcp1;
mcp23xxx mcp2;

void setup()
{
//  Serial.begin(38400);
//  Serial.println("start");
  mcp1.beginGPIO(MCP_CSPIN,0x20,MCP23S17);
  mcp2.beginGPIO(MCP_CSPIN,0x21,MCP23S17);
  // Set all pins to be outputs (by default they are all inputs)
  mcp1.gpioPinMode(OUTPUT);
  mcp2.gpioPinMode(OUTPUT);

  // Change all pins at once, 16-bit value
  mcp1.gpioPort(0xFFFF);
  mcp2.gpioPort(0xFFFF);

}

void loop()
{
  mcp1.gpioPort(0xFFFF);
  mcp2.gpioPort(0xFFFF);
  for (int i=0;i<16;i++){
    mcp1.gpioDigitalWrite(i,LOW);
    mcp2.gpioDigitalWrite(16-i,LOW);
    delay(40);
  }
}