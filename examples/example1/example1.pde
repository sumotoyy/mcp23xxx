#include <SPI.h>
#include <mcp23xxx.h>

#define MCP_CSPIN  10 

mcp23xxx mcp;

void setup()
{
  Serial.begin(38400);
  Serial.println("start");
  mcp.beginGPIO(MCP_CSPIN,0x20,MCP23S17);//must exists!
  // Set all pins to be outputs (by default they are all inputs)
  mcp.gpioPinMode(OUTPUT);

  // Change all pins at once, 16-bit value
  mcp.gpioPort(0xFFFF);

}

void loop()
{
  for (int i=0;i<16;i++){
    mcp.gpioDigitalWrite(i,LOW);
    delay(50);
  }
  for (int i=0;i<16;i++){
    mcp.gpioDigitalWrite(i,HIGH);
    delay(50);
  }

}