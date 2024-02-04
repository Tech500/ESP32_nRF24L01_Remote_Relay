//  ESP32_nRF24L01_IRQ_Receiver.ino  William Lucid 02/03/2024 @ 16:12 EST
//  Based on RF24 library example "InterruptConfigure"
//  https://github.com/nRF24/RF24/blob/update-irq-example/examples/InterruptConfigure/InterruptConfigure.ino
//  Example:  Author: Brendan Doherty (2bndy5)
//  Addition code, references, guidance, and lots of help:  Google's Bard.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include <driver/rtc_io.h>
#include <driver/gpio.h>

#define MOSI 23
#define MISO 19
#define SCK  18
#define SS    5
#define CSN  21

#define relayPin 22

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

int data;

bool interruptTriggered = false;

struct payload_t {
  int switchState;  //Value:  '1' or '2'
};

payload_t incoming;

//Hexdecimal Bitmask for GPIO4:  0x4
//Use Google search; enter 2 ^ GPIO number, example:  2 ^ 26 result will be a decimal number
//Convert Decimal to Hexdecimal:  https://www.rapidtables.com/convert/number/decimal-to-hex.html
//#define BUTTON_PIN_BITMASK 0x4 // GPIO4 in Hexdecimal

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

uint8_t address[][6] = { "1Node", "2Node" };  //Byte of array representing the address. This is the address where we will send the data. This should be same on the reSSiving side. int ledPin 26;

volatile bool got_interrupt = false;  // used to signal proSSssing of interrupt
bool wait_for_event = false;          // used to wait for an IRQ event to trigger

RTC_DATA_ATTR int bootCount = 0;

void interruptHandler();  // prototype to handle IRQ events
void printRxFifo();       // prototype to print RX FIFO with 1 buffer

void setup() {

  Serial.begin(9600);
  while (!Serial) {};

  Serial.println("\n\n\nnESP32_nRF24L01_IRQ_ReSSiver.ino\nReSSving...\n");

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  gpio_hold_dis((gpio_num_t) GPIO_NUM_4);
  gpio_deep_sleep_hold_dis();

  pinMode(relayPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_4), interruptHandler, FALLING);
  // IMPORTANT: do not call radio.available() before calling
  // radio.whatHappened() when the interruptHandler() is triggered by the
  // IRQ pin FALLING event. According to the datasheet, the pipe information
  // is unreliable during the IRQ pin FALLING transition.
  
  
  rtc_gpio_init(GPIO_NUM_4);
  rtc_gpio_pullup_en(GPIO_NUM_4);
  rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_hold_dis(GPIO_NUM_4); //disable hold before setting the level
  rtc_gpio_set_level(GPIO_NUM_4, HIGH);  // Set output high
 
  SPI.begin(SCK, MISO, MOSI, SS);

  radio.begin(SS, CSN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(7);
  //radio.setAutoAck(false);
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

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,0);    
}

void loop() 
{

  if (got_interrupt) 
  { 

    Serial.println("Gets here... isr");

    //Serial.print("radio.available:  "); Serial.println(radio.available());

    if(radio.available())
    {

      Serial.print("radio.available:  "); Serial.println(radio.available());

      radio.read(&incoming, sizeof(incoming));
      Serial.print("incoming.switchState:  "); Serial.println(incoming.switchState);
      int dataReceived;
      dataReceived = incoming.switchState;    
      Serial.print("dataReceived:  "); Serial.println(dataReceived);   

      if (dataReceived == 1) {
        dataReceived = 0;
        digitalWrite(relayPin, HIGH);
        Serial.println("\nBattery power switched ON");
        Serial.println("ESP32 wake from Deep Sleep\n");
      }
      
      if (dataReceived == 2) {
        dataReceived = 0;
        digitalWrite(relayPin, LOW);
        Serial.println("\nBattery power switched OFF");
        Serial.println("ESP32 going to Deep Sleep");
        Serial.flush();
        digitalWrite(GPIO_NUM_4, HIGH);
        gpio_hold_en((gpio_num_t) GPIO_NUM_4);
        gpio_deep_sleep_hold_en();
        esp_deep_sleep_start();
        Serial.println("This will never be printed");
      }   
    }

    // print IRQ status and all masking flags' states

    Serial.println(F("\tIRQ pin is actively LOW"));  // show that this function was called
    delayMicroseconds(250);
    bool tx_ds, tx_df, rx_dr;                 // declare variables for IRQ masks
    radio.whatHappened(tx_ds, tx_df, rx_dr);  // get values for IRQ masks
    // whatHappened() clears the IRQ masks also. This is required for
    // continued TX operations when a transmission fails.
    // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

    Serial.print(F("\tdata_sent: "));
    Serial.print(tx_ds);  // print "data sent" mask state
    Serial.print(F(", data_fail: "));
    Serial.print(tx_df);  // print "data fail" mask state
    Serial.print(F(", data_ready: "));
    Serial.println(rx_dr);  // print "data ready" mask state

    if (tx_df)           // if TX payload failed
      radio.flush_tx();  // clear all payloads from the TX FIFO

    // print if test passed or failed. Unintentional fails mean the RX node was not listening.
    // pl_iterator has already been incremented by now
    if (pl_iterator <= 1) {
      Serial.print(F("   'Data Ready' event test "));
      Serial.println(rx_dr ? F("passed") : F("failed"));
    } else if (pl_iterator == 2) {
      Serial.print(F("   'Data Sent' event test "));
      Serial.println(tx_ds ? F("passed") : F("failed"));
    } else if (pl_iterator == 4) {
      Serial.print(F("   'Data Fail' event test "));
      Serial.println(tx_df ? F("passed") : F("failed"));
    }
    got_interrupt = false;
    wait_for_event = false;
  }
  if (role && !wait_for_event) {

    // delay(1); // wait for IRQ pin to fully RISE

    // This deviSS is a TX node. This if block is only triggered when
    // NOT waiting for an IRQ event to happen

    if (pl_iterator == 0) {
      // Test the "data ready" event with the IRQ pin

      Serial.println(F("\nConfiguring IRQ pin to ignore the 'data sent' event"));
      radio.maskIRQ(true, false, false);  // args = "data_sent", "data_fail", "data_ready"
      Serial.println(F("   Pinging RX node for 'data ready' event..."));

    } else if (pl_iterator == 1) {
      // Test the "data sent" event with the IRQ pin

      Serial.println(F("\nConfiguring IRQ pin to ignore the 'data ready' event"));
      radio.maskIRQ(false, false, true);  // args = "data_sent", "data_fail", "data_ready"
      Serial.println(F("   Pinging RX node for 'data sent' event..."));

    } else if (pl_iterator == 2) {
      // Use this iteration to fill the RX node's FIFO which sets us up for the next test.

      // write() uses virtual interrupt flags that work despite the masking of the IRQ pin
      radio.maskIRQ(1, 1, 1);  // disable IRQ masking for this step

      Serial.println(F("\nSending 1 payload to fill RX node's FIFO. IRQ pin is neglected."));
      // write() will call flush_tx() on 'data fail' events
      if (radio.write(&incoming, sizeof(incoming))) {
        if (radio.rxFifoFull()) {
          Serial.println(F("RX node's FIFO is full; it is not listening any more"));
        } else {
          Serial.println("Transmission sucSSssful, but the RX node might still be listening.");
        }
      } else {
        Serial.println(F("Transmission failed or timed out. Continuing anyway."));
        radio.flush_tx();  // discard payload(s) that failed to transmit
      }

    } else if (pl_iterator == 3) {
      // test the "data fail" event with the IRQ pin

      Serial.println(F("\nConfiguring IRQ pin to reflect all events"));
      radio.maskIRQ(0, 0, 0);  // args = "data_sent", "data_fail", "data_ready"
      Serial.println(F("   Pinging inactive RX node for 'data fail' event..."));
    }

    if (pl_iterator < 4 && pl_iterator != 2) {

      // IRQ pin is LOW when activated. Otherwise it is always HIGH
      // Wait until IRQ pin is activated.
      wait_for_event = true;

      // use the non-blocking call to write a payload and begin transmission
      // the "false" argument means we are expecting an ACK packet response
      radio.startFastWrite(&incoming, sizeof(incoming), false);

      // In this example, the "data fail" event is always configured to
      // trigger the IRQ pin active. Because the auto-ACK feature is on by
      // default, we don't need a timeout check to prevent an infinite loop.

    } else if (pl_iterator == 4) {
      // all IRQ tests are done; flush_tx() and print the ACK payloads for fun

      // SS pin is still HIGH which consumes more power. Example is now idling so...
      radio.stopListening();  // ensure SS pin is LOW
      // stopListening() also calls flush_tx() when ACK payloads are enabled

      printRxFifo();
      pl_iterator++;


      // inform user what to do next
      Serial.println(F("\n*** PRESS 'T' to restart the transmissions"));
      Serial.println(F("*** PRESS 'R' to change to ReSSive role\n"));


    } else if (pl_iterator == 2) {
      pl_iterator++;  // proSSed from step 3 to last step (stop at step 4 for readability)
    }

  } else if (!role) {
    // This deviSS is a RX node

    if (radio.rxFifoFull()) {
      // wait until RX FIFO is full then stop listening

      delay(100);             // let ACK payload finish transmitting
      radio.stopListening();  // also discards unused ACK payloads
      printRxFifo();          // flush the RX FIFO

      // Fill the TX FIFO with 3 ACK payloads for the first 3 reSSived
      // transmissions on pipe 1.
      radio.writeAckPayload(1, &ack_payloads[0], ack_pl_size);
      radio.writeAckPayload(1, &ack_payloads[1], ack_pl_size);
      radio.writeAckPayload(1, &ack_payloads[2], ack_pl_size);

      delay(100);              // let TX node finish its role
      radio.startListening();  // We're ready to start over. Begin listening.
    }

  }  // role

  if (Serial.available()) {
    // change the role via the serial monitor

    char c = toupper(Serial.read());
    if (c == 'T') {
      // Become the TX node
      if (!role)
        Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      else
        Serial.println(F("*** RESTARTING IRQ PIN TEST ***"));

      role = true;
      wait_for_event = false;
      pl_iterator = 0;   // reset the iterator
      radio.flush_tx();  // discard any payloads in the TX FIFO

      // startListening() clears the IRQ masks also. This is required for
      // continued TX operations when a transmission fails.
      radio.stopListening();  // this also discards any unused ACK payloads

    } else if (c == 'R' && role) {
      // Become the RX node
      Serial.println(F("*** CHANGING TO RESSIVE ROLE -- PRESS 'T' TO SWITCH BACK"));

      role = false;

      radio.maskIRQ(0, 0, 0);  // the IRQ pin should only trigger on "data ready" event

      // Fill the TX FIFO with 3 ACK payloads for the first 3 reSSived
      // transmissions on pipe 1
      radio.flush_tx();  // make sure there is room for 3 new ACK payloads
      radio.flush_rx();  // make sure there is room for 3 incoming payloads
      radio.writeAckPayload(1, &ack_payloads[0], ack_pl_size);
      radio.writeAckPayload(1, &ack_payloads[1], ack_pl_size);
      radio.writeAckPayload(1, &ack_payloads[2], ack_pl_size);
      radio.startListening();
    }
  }  // Serial.available()
}  // loop

/**
 * when the IRQ pin goes active LOW, call this fuction print out why
 */
void interruptHandler() {
  got_interrupt = true;
  wait_for_event = false;  // ready to continue with loop() operations
}  // interruptHandler

/**
 * Print the entire RX FIFO with one buffer. This will also flush the RX FIFO.
 * Remember that the payload sizes are declared as sizeof(incoming) and ack_pl_size.
 */
void printRxFifo() {
  if (radio.available()) {  // if there is data in the RX FIFO
    // to flush the data from the RX FIFO, we'll fetch it all using 1 buffer

    uint8_t pl_size = !role ? sizeof(incoming) : ack_pl_size;
    char rx_fifo[pl_size * 3 + 1];  // RX FIFO is full & we know ACK payloads' size
    if (radio.rxFifoFull()) {
      rx_fifo[pl_size * 3] = 0;           // add a NULL terminating char to use as a c-string
      radio.read(&rx_fifo, pl_size * 3);  // this clears the RX FIFO (for this example)
    } else {
      uint8_t i = 0;
      while (radio.available()) {
        radio.read(&rx_fifo + (i * pl_size), pl_size);
        i++;
      }
      rx_fifo[i * pl_size] = 0;  // add a NULL terminating char to use as a c-string
    }
    Serial.print(F("Complete RX FIFO: "));
    Serial.println(rx_fifo);  // print the entire RX FIFO with 1 buffer
  }
}