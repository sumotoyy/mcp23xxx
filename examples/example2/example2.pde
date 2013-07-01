
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
  mcp.gpioPort(0b0101010101010101);

  // Write to individual pins
  mcp.gpioDigitalWrite(8,LOW);
  mcp.gpioDigitalWrite(12,HIGH);

  // Read all pins at once, 16-bit value
  uint16_t pinstate = mcp.gpioPort();

  // Set pin 14 (GPIO B6) to be an input
  mcp.gpioPinMode(14,INPUT);

  // Read the value of pin 14
  mcp.gpioDigitalRead(14);
}

// Cycle the output lines at a staggered rate, pin/2 milliseconds
long timeoutInterval = 1;
long previousMillis = 0;
uint16_t counter = 0x0000;

void timeout()
{
  mcp.port(counter++);
}  

void loop()
{
  // handle timeout function, if any
  if (millis() - previousMillis > timeoutInterval)
  {
    timeout();
    previousMillis = millis();
  }
}