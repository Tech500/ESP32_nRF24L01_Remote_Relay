//  ESP32_nRF24L01_IRQ_Receiver.ino  William Lucid 02/14/2024 @ 08:45 AM EST
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
#include <driver/rtc_io.h>
#include <driver/gpio.h>

#define MOSI 23
#define MISO 19
#define SCK  18
#define SS   22
#define CSN  15

#define INTERRUPT_PIN GPIO_NUM_33

#define INTERRUPT_PIN_BITMASK 0x100000000 // 2^33 in hex

#define relayPin 21

RTC_DATA_ATTR int bootCount = 0;

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

int data;

bool irqFlag = false;

struct payload_t {
  int switchState;  
};

payload_t incoming;

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or reSSiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a string that changes on every transmission. (sucSSssful or not)
// Make a couple arrays of payloads & an iterator to traverse them
const uint8_t tx_pl_size = 5;
const uint8_t ack_pl_size = 4;
uint8_t pl_iterator = 0;
// The " + 1" compensates for the c-string's NULL terminating 0
//char tx_payloads[][sizeof(incoming) + 1] = { "Ping ", "Pong ", "Radio", "1FAIL" };
char ack_payloads[][ack_pl_size + 1] = { "Yak ", "Back", " ACK" };

RF24 radio(SS, CSN, 4000000);  // set SPI speed to 4MHz instead of default 10MHz

uint8_t address[][6] = { "1Node", "2Node" };  //Byte of array representing the address. This is the address where we will send the data. This should be same on the reSSiving side. 

void IRAM_ATTR gpio_isr_handler(void* arg) {
  irqFlag = true;  // Set the flag here
}  

void interruptHandler();  // prototype to handle IRQ events
//void printRxFifo();       // prototype to print RX FIFO with 1 buffer

void setup() {

  Serial.begin(9600);
  while (!Serial) {};

  Serial.println("\n\n\nnESP32_nRF24L01_IRQ_Receiver.ino\nReceiving...\n");

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("\n\nBoot number: " + String(bootCount));

  Serial.println("\nCPU0 reset reason:");
  print_reset_reason(rtc_get_reset_reason(0));
  //verbose_print_reset_reason(rtc_get_reset_reason(0));

  Serial.println("CPU1 reset reason:");
  print_reset_reason(rtc_get_reset_reason(1));
  //verbose_print_reset_reason(rtc_get_reset_reason(1));

  rtc_gpio_hold_dis(INTERRUPT_PIN);

  //pinMode(CSN, INPUT);
  //digitalWrite(CSN, LOW);

  pinMode(relayPin, OUTPUT);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, GPIO_INTR_LOW_LEVEL); 

  // Configure pin for interrupt  ... in setup() or elsewhere
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge (LOW)
  io_conf.pin_bit_mask = (1ULL << INTERRUPT_PIN);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable internal pullup resistor
  gpio_config(&io_conf);

  gpio_install_isr_service(0);  // Install GPIO interrupt service
  gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, (void*)INTERRUPT_PIN);

  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, FALLING);
  // IMPORTANT: do not call radio.available() before calling
  // radio.whatHappened() when the interruptHandler() is triggered by the
  // IRQ pin FALLING event. According to the datasheet, the pipe information
  // is unreliable during the IRQ pin FALLING transition.

  rtc_gpio_init(INTERRUPT_PIN);
  rtc_gpio_pullup_en(INTERRUPT_PIN);
  rtc_gpio_set_direction(INTERRUPT_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_hold_dis(INTERRUPT_PIN); //disable hold before setting the level
  rtc_gpio_set_level(INTERRUPT_PIN, HIGH);  // Set output high
  rtc_gpio_hold_en(INTERRUPT_PIN);

  SPI.begin(SCK, MISO, MOSI, SS);

  radio.begin(SS, CSN);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.setChannel(7);
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

  // Fill the TX FIFO with 3 ACK payloads for the first 3 reSSived
  // transmissions on pipe 1
  radio.writeAckPayload(1, &ack_payloads[0], ack_pl_size);
  radio.writeAckPayload(1, &ack_payloads[1], ack_pl_size);
  radio.writeAckPayload(1, &ack_payloads[2], ack_pl_size);

  radio.startListening();
  
  // For debugging info
  // printf_begin();             // needed only onSS for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
}

void loop() 
{

  for(int x = 1;x < 2;x++){  
    Serial.println("\nNo deep sleep\n");
    while(irqFlag != true){}
  }
  
  if (irqFlag == true) 
  { 

    irqFlag = false;
    
    bool tx_ds, tx_df, rx_dr;                 // declare variables for IRQ masks
    radio.whatHappened(tx_ds, tx_df, rx_dr);  // get values for IRQ masks

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
        Serial.println("ESP32 going to Deep Sleep");
        //digitalWrite(INTERRUPT_PIN, HIGH);
        delay(100);  //Required to get ready
        deepSleep();
        Serial.println("This will never be printed");
      }   
    }
  }
}

void deepSleep(){
  rtc_gpio_init(INTERRUPT_PIN);
  rtc_gpio_pullup_en(INTERRUPT_PIN); 
  rtc_gpio_pulldown_dis(INTERRUPT_PIN);
  gpio_deep_sleep_hold_en();
  gpio_hold_en(INTERRUPT_PIN);  
  esp_deep_sleep_start();
}
