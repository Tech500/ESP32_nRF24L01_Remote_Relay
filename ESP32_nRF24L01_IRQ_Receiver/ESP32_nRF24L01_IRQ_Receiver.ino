//  ESP32_nRF24L01_IRQ_Receiver.ino  William Lucid 02/04/2024 @ 21:07 EST
//  Based on RF24 library example "InterruptConfigure"
//  https://github.com/nRF24/RF24/blob/update-irq-example/examples/InterruptConfigure/InterruptConfigure.ino
//  Example:  Author: Brendan Doherty (2bndy5)
//  Addition code, references, guidance, and lots of help:  Google's Bard.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#include "esp_sleep.h"
#include <driver/rtc_io.h>

#define MOSI 23
#define MISO 19
#define SCK  18
#define SS    5
#define CSN  21

#define INTERRUPT_PIN GPIO_NUM_33

#define relayPin 22

#define POLL_INTERVAL_MS 100 // Example poll interval in milliseconds

volatile bool rtc_interrupt_triggered = false;
volatile bool ext_wakeup_triggered = false;
bool external_wakeup = false; // Set to true if external wakeup is enabled

void IRAM_ATTR rtc_isr() {
  rtc_interrupt_triggered = true;
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

  pinMode(relayPin, OUTPUT);

  // Setup RTC GPIO as input  
  rtc_gpio_init(INTERRUPT_PIN);
  rtc_gpio_set_direction(INTERRUPT_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_level(INTERRUPT_PIN, LOW);    
  rtc_gpio_pullup_en(INTERRUPT_PIN);  
  
  // Attach interrupt for RTC GPIO pin
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rtc_isr, CHANGE);
  
  SPI.begin(SCK, MISO, MOSI, SS);

  radio.begin(SS, CSN);
  radio.setDataRate(RF24_250KBPS);
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

  /*
  // For debugging info
   printf_begin();             // needed only onSS for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
   radio.printPrettyDetails(); // (larger) function that prints human readable data
  */

  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 0);

  radio.startListening();
}

void loop() 
{

  if (rtc_interrupt_triggered) {

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
      }   
    }

    rtc_interrupt_triggered = false;

  }
}

void gotoDeepsleep(){

        Serial.println("Gets to gotoDeepsleep");
  
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        rtc_gpio_set_direction(INTERRUPT_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_set_level(INTERRUPT_PIN, HIGH);  // Set output high       
        rtc_gpio_pullup_en(INTERRUPT_PIN); // Correct argument
        rtc_gpio_pulldown_dis(INTERRUPT_PIN);
        rtc_gpio_hold_en(INTERRUPT_PIN);
        esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, LOW);  
        delay(100);
        esp_deep_sleep_start();
        Serial.println("This will never be printed");
}