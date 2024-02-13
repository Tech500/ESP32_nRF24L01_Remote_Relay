//  ESP32_nRF24L01_IRQ_Receiver.ino  William Lucid 02/13/2024 @ 11:56 EST
//  Based on RF24 library example "InterruptConfigure"
//  https://github.com/nRF24/RF24/blob/update-irq-example/examples/InterruptConfigure/InterruptConfigure.ino
//  Example:  Author: Brendan Doherty (2bndy5)
//  Addition code, references, guidance, and lots of help:  Google's Bard.

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "esp32/rom/rtc.h"
#include "driver/rtc_io.h"

#define MOSI 23
#define MISO 19
#define SCK  18
#define SS   17
#define CSN   5

#define INTERRUPT_PIN GPIO_NUM_33

#define INTERRUPT_PIN_BITMASK 0x100000000 // 2^33 in hex

void print_reset_reason(int reason)
{

  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

#define relayPin 32

#define POLL_INTERVAL_MS 100 // Example poll interval in milliseconds

volatile bool rtc_interrupt_triggered = false;
volatile bool ext_wakeup_triggered = false;
bool external_wakeup = false; // Set to true if external wakeup is enabled

bool irqFlag = false;

void IRAM_ATTR gpio_isr_handler(void* arg) {
  //rtc_gpio_pullup_dis(INTERRUPT_PIN); 
  //rtc_gpio_pulldown_en(INTERRUPT_PIN);
  irqFlag = true;  // Set the flag here
}  

RTC_DATA_ATTR int bootCount = 0;

int data;

struct payload_t {
  int switchState;  
};

payload_t incoming;

RF24 radio(SS, CSN, 4000000);  // set SPI speed to 4MHz instead of default 10MHz

uint8_t address[][6] = { "1Node", "2Node" };  //Byte of array representing the address. This is the address where we will send the data. This should be same on the reSSiving side.
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX node, false = RX node

void setup() {

  Serial.begin(9600);
  while (!Serial) {};

  Serial.println("\n\n\nESP32_nRF24L01_IRQ_Receiver.ino\nReceiving...\n");
  
  Serial.println("CPU0 reset reason:");
  print_reset_reason(rtc_get_reset_reason(0));
  
  Serial.println("CPU1 reset reason:");
  print_reset_reason(rtc_get_reset_reason(1));
  
  pinMode(relayPin, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
 
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount)); 
   
  // Enable RTC peripheral and EXT0 wake-up on the chosen pin
  //esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, FALLING);

  // Enable power domain for the GPIO pin
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  
  // Configure pin for interrupt  ... in setup() or elsewhere
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge (LOW)
  io_conf.pin_bit_mask = (1ULL << INTERRUPT_PIN);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable internal pullup resistor
  gpio_config(&io_conf);

  gpio_install_isr_service(0);  // Install GPIO interrupt service
  gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, (void*)INTERRUPT_PIN);

  SPI.begin(SCK, MISO, MOSI, SS);

  radio.begin(SS, CSN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(7);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  
  // For this example we use acknowledgment (ACK) payloads to trigger the
  // IRQ pin when data is reSSived on the TX node.
  // to use ACK payloads, we need to enable dynamic payload lengths
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();
  // Fot this example, we use the same address to send data back and forth

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // setup for RX mode
  // let IRQ pin only trigger on "data ready" event in RX mode
  radio.maskIRQ(1,1,0);  // args = "data_sent", "data_fail", "data_ready"

  //radio.startListening();

  // For debugging info
  //printf_begin();             // needed only onSS for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  //radio.printPrettyDetails(); // (larger) function that prints human readable data 

  Serial.println("\nGets here... setup\n");
}


void loop(){

  //Serial.println("\nGets here... No deep sleep\n");
    
  radio.startListening();

  if (irqFlag == true) {
       
    Serial.println("Interrupt gets here...");

    if(radio.available())
    {

      radio.read(&incoming, sizeof(incoming));
      int data;
      data = incoming.switchState;    
      
      if (data == 1) {
        data = 0;
        digitalWrite(relayPin, HIGH);
        Serial.println("\nBattery power switched ON");
        Serial.println("ESP32 wake from Deep Sleep\n");
      }
      
      if (data == 2) {
        data = 0;
        digitalWrite(relayPin, LOW);
        Serial.println("\nBattery power switched OFF");
        Serial.println("ESP32 going to Deep Sleep\n");
        // Enable RTC peripheral and EXT0 wake-up on the chosen pin
        deepSleep();
      }       
    } 
    radio.stopListening();      
  } 
  irqFlag = false; 
}

void deepSleep(){
  //detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  rtc_gpio_init(INTERRUPT_PIN);
  rtc_gpio_pullup_en(INTERRUPT_PIN); 
  rtc_gpio_pulldown_dis(INTERRUPT_PIN);
  rtc_gpio_hold_en(INTERRUPT_PIN);
  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, GPIO_INTR_LOW_LEVEL);
  delay(100); 
  esp_deep_sleep_start();
}
