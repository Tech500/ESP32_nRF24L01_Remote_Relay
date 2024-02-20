ESP32 DEVKIT V1:  ESP32_nRF24L01_Remote_Relay                 Latest!  02/20/2024 @ 18:09 EST

Uses external 0, RTC_IO,  wake-up by 2.4 Ghz., RF Interrupt for a Deep Sleeping, ESP32. n24L01, transceiver; code uses two, nRF24L01 to remotely switch battery on or off for a low demand, 
video live stream. Web camera consumes 10,000 mAH every 24 hours; switching battery on/off by webserver, URL Request over the nRF24L01 should extend battery between battery charges.

Thanks to Google's Gemini for answering my many prompts and Gemini's guidence; especially with RTC_IO, Bard helped make this project a reality!

02/20/2024 Files have been updated; supporting switching battery on/off, putting the ESP32 into Deep Sleep and Wakeup from Deep Sleep..

SPI-ESP32 pin connections for ESP32 DEVKIT V1 board:

MOSI 23, MISO 19, SCK 18, CE 12 CSN 17 IRQ 33, Vcc, and GND.  Pins were verified by successfully running RF24 library's "Getting Started" example code.

Project is a work-in-progress...  No attempt to reduce current consumption...

Connection to n24L01, IRQ directly to GPIO_NUM_33

Developed using Chrome browser, Arduino IDE 2.2.1, ESP32 Core 2.0.14, and ESP32 Devkit v1 Development board. 


William
