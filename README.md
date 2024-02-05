ESP32 DEVKIT V1:  ESP32_nRF24L01_Remote_Relay                 Latest!

Uses external wake-up by 2.4 Ghz., RF Interrupt for a Deep Sleeping, ESP32. n24L01, transceiver; code uses two, nRF24L01 to remotely switch battery on or off for a low demand, 
video live stream. Web camera consumes 10,000 mAH every 24 hours; switching battery on/off by webserver, URL Request over the nRF24L01 should extend battery between battery charges.

Thanks to Google's Bard for answering my many prompts and Bard's guidence; especially with RTC_IO, Bard helped make this project a reality!

02/04/2024 Files have been updated; supporting switching battery on/off, putting the ESP32 into Deep Sleep, and waking ESP32 from Deep Sleep; using external 0, RTC_GPIO pin.

SPI-ESP32 pin connections for ESP32 DEVKIT V1 board:

MOSI 23, MISO 19, SCK 18, CE 5 CSN 21 IRQ 4, Vcc, and GND.  Pins were verified by successfully running RF24 library's "Getting Started" example code.

Project is a work-in-progress...  No attempt to reduce current consumption...

William
