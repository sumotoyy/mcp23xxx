mcp23xxx version 1.0b2
=======================

Microchip MCP23xxxx GPIO extender library

This library has been developed for drive most MCP23xxxx chip from Microchip. It support all features like HAEN
(that is unique to Microchip and allows 8 chips share the same CS pin with the SPI version). I still not developed the IRQ
features due some strange difference from I2C and SPI version probably due an hardware bug of the chip so I need
some further investigation to fix it. I choosed to build my version because I found many different libraries but none 
worked as I expected.
This Library can be easily included in other libraries.

Chip Supported:

	MCP23S17
	MCP23017
	MCP23S08
	MCP2308
	MCP23S18
	MCP23018
	MCP23016
	MCP2309
