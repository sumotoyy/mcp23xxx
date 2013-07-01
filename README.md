mcp23xxx
========

Microchip MCP23xxxx GPIO extender library

This library has been developed for drive most MCP23xxxx chip from Microchip. It support all features like HAEN
(that is unique to Microchip and allows 8 chips share the same CS pin with the SPI version). I still not developed the IRQ
features due some strange difference from I2C and SPI version probably due an hardware bug of the chip so I need
some further investigation to fix it. I choosed to build my version because I found many different libraries but none 
worked as I expected.
