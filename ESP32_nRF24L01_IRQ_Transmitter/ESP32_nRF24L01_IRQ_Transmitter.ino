//  ESP32_nRF24L01_IRQ_Transmitter.ino  William Lucid 02/03/2024 @ 16:12 EST
//  Based on RF24 library example "InterruptConfigure"
//  https://github.com/nRF24/RF24/blob/update-irq-example/examples/InterruptConfigure/InterruptConfigure.ino
//  Example:  Author: Brendan Doherty (2bndy5)
//  Addition code, references, guidance, and lots of help:  Google's Bard.

//  See library downloads for each library license.

#include "WiFi.h"
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <SPI.h>
#include "printf.h"
#include <nRF24L01.h>
#include <RF24.h>  //https://github.com/nRF24/RF24
#include <Ticker.h>

#import "index7.h"  //Video feed HTML; do not remove

#define MOSI 23
#define MISO 19
#define SCK 18
#define SS 5
#define CSN 21

#define IRQ_PIN 4

// Replace with your network details
const char *ssid = "#####";
const char *password = "XXXXXXX";

WiFiClient client;

AsyncWebServer server(80);

int data;

volatile bool got_interrupt = false;  // used to signal processing of interrupt
bool wait_for_event = false;          // used to wait for an IRQ event to trigger

//Hexdecimal Bitmask for GPIO4:  4
//Use Google search; enter 2 ^ GPIO number, example:  2 ^ 26 result will be a decimal number
//Convert Decimal to Hexdecimal:  https://www.rapidtables.com/convert/number/decimal-to-hex.html
//#define BUTTON_PIN_BITMASK 0x4  

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

uint8_t address[][6] = { "1Node", "2Node" };  //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side. int ledPin 26;

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX node, false = RX node

// For this example, we'll be using a payload containing
// a string that changes on every transmission. (successful or not)
// Make a couple arrays of payloads & an iterator to traverse them
const uint8_t tx_pl_size = 5;
const uint8_t ack_pl_size = 4;
uint8_t pl_iterator = 0;
// The " + 1" compensates for the c-string's NULL terminating 0
//char tx_payloads[][tx_pl_size + 1] = { "Ping ", "Pong ", "Radio", "1FAIL" };
char ack_payloads[][ack_pl_size + 1] = { "Yak ", "Back", " ACK" };

RF24 radio(SS, CSN, 4000000);  // set SPI speed to 4MHz instead of default 10MHz

void interruptHandler();  //proto type to handle IRQ events
void printRxFifo();       //proto type to print RX FIFO with 1 buffer

// Define payload structure for switch control variable
struct payload_t {
  int switchState;
};

payload_t payload;

Ticker oneTick;
Ticker onceTick;

String linkAddress = "xxx.xxx.xxx.xxx:80";

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int watchdogCounter;
int totalwatchdogCounter;
int cameraPowerOff = 0;
int watchDog;

void ISRwatchdog() {

  portENTER_CRITICAL_ISR(&mux);

  watchdogCounter++;

  if (watchdogCounter >= 75) {

    watchDog = 1;
  }

  portEXIT_CRITICAL_ISR(&mux);
}

int cameraFlag;
int needAnotherCountdown = 0;

void ISRcamera() {
  int data = 2;
  switchOne(data);
  onceTick.detach();  //only run countDown timer once
}

/**
 * when the IRQ pin goes active LOW, call this fuction print out why
 */
void interruptHandler() {
  got_interrupt = true;
  wait_for_event = false;  // ready to continue with loop() operations
}  // interruptHandler

/**
 * Print the entire RX FIFO with one buffer. This will also flush the RX FIFO.
 * Remember that the payload sizes are declared as tx_pl_size and ack_pl_size.
 */
void printRxFifo() {
  if (radio.available()) {  // if there is data in the RX FIFO
    // to flush the data from the RX FIFO, we'll fetch it all using 1 buffer

    uint8_t pl_size = !role ? tx_pl_size : ack_pl_size;
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



void setup() {

  Serial.begin(9600);
  while (!Serial) {};

  Serial.println("\n\n\nWebserver and");
  Serial.println("nRF24L01 Transmitter for turning ON Videofeed\n");

  wifi_Start();

  //nRF24L01

  SPI.begin(SCK, MISO, MOSI, SS);

  radio.begin(SS, CSN);  //Starting radio the Wireless communication
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(7);
  //radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.maskIRQ(0, 1, 0);  //Enable TX_DS interrupt

  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), interruptHandler, FALLING);
  // IMPORTANT: do not call radio.available() before calling
  // radio.whatHappened() when the interruptHandler() is triggered by the
  // IRQ pin FALLING event. According to the datasheet, the pipe information
  // is unreliable during the IRQ pin FALLING transition.

  // For this example we use acknowledgment (ACK) payloads to trigger the
  // IRQ pin when data is received on the TX node.
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

  radio.setPayloadSize(sizeof(payload));  // Set payload size

  // additional setup specific to the node's role
  if (role) {
    //setup for TX mode
    radio.stopListening();  // put radio in TX mode

  } else {
    // setup for RX mode

    // let IRQ pin only trigger on "data ready" event in RX mode
    radio.maskIRQ(1, 1, 0);  // args = "data_sent", "data_fail", "data_ready"

    // Fill the TX FIFO with 3 ACK payloads for the first 3 received
    // transmissions on pipe 1
    radio.writeAckPayload(1, &ack_payloads[0], ack_pl_size);
    radio.writeAckPayload(1, &ack_payloads[1], ack_pl_size);
    radio.writeAckPayload(1, &ack_payloads[2], ack_pl_size);

    radio.startListening();  // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, PSTR("text/html"), HTML7, processor7);

    end();

    needAnotherCountdown = 1;
    countdownTrigger();
  });

  server.begin();

  oneTick.attach(1.0, ISRwatchdog);  //watchdog  ISR triggers every 1 second

  radio.stopListening();  //This sets the module as transmitter
}

void loop() {
  if (got_interrupt) {
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

    // This device is a TX node. This if block is only triggered when
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
      if (radio.write(&payload, sizeof(payload))) {
        if (radio.rxFifoFull()) {
          Serial.println(F("RX node's FIFO is full; it is not listening any more"));
        } else {
          Serial.println("Transmission successful, but the RX node might still be listening.");
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
      radio.startFastWrite(&payload, sizeof(payload), false);

      // In this example, the "data fail" event is always configured to
      // trigger the IRQ pin active. Because the auto-ACK feature is on by
      // default, we don't need a timeout check to prevent an infinite loop.

    } else if (pl_iterator == 4) {
      // all IRQ tests are done; flush_tx() and print the ACK payloads for fun

      // CE pin is still HIGH which consumes more power. Example is now idling so...
      radio.stopListening();  // ensure CE pin is LOW
      // stopListening() also calls flush_tx() when ACK payloads are enabled

      printRxFifo();
      pl_iterator++;

      // inform user what to do next
      Serial.println(F("\n*** PRESS 'T' to restart the transmissions"));
      Serial.println(F("*** PRESS 'R' to change to Receive role\n"));


    } else if (pl_iterator == 2) {
      pl_iterator++;  // proceed from step 3 to last step (stop at step 4 for readability)
    }

  } else if (!role) {
    // This device is a RX node

    if (radio.rxFifoFull()) {
      // wait until RX FIFO is full then stop listening

      delay(100);             // let ACK payload finish transmitting
      radio.stopListening();  // also discards unused ACK payloads
      printRxFifo();          // flush the RX FIFO

      // Fill the TX FIFO with 3 ACK payloads for the first 3 received
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
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));

      role = false;

      radio.maskIRQ(0, 0, 0);  // the IRQ pin should only trigger on "data ready" event

      // Fill the TX FIFO with 3 ACK payloads for the first 3 received
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

String processor7(const String &var) {

  //index7.h

  if (var == F("LINK"))
    return linkAddress;

  return String();
}

void countdownTrigger() {
  // Perform countdown actions here
  Serial.println("\nCountdown timer triggered!\n");
  // Schedule the next countdown if needed
  if (needAnotherCountdown == 1) {
    onceTick.once(20, ISRcamera);
    int data = 1;
    switchOne(data);
    needAnotherCountdown = 0;
  }
}

void switchOne(int data) {

  radio.stopListening();

  if (data == 1) {
    payload.switchState = 1;
    radio.write(&payload, sizeof(payload));  //Sending data = 1
    Serial.println("Battery Switch is ON");
    Serial.println("ESP32 waking from Deep Sleep\n");
  }

  if (data == 2) {
    payload.switchState = 2;
    radio.write(&payload, sizeof(payload));  //Sending data = 2
    Serial.println("Battery power switched OFF");
    Serial.println("ESP32 going to Deep Sleep\n");
  }

  data = 0;
}

void end() {

  delay(1000);

  Serial.println("");
  Serial.println("");
}

void wifi_Start() {

//Server settings
#define ip { 10, 0, 0, 27 }
#define subnet \
  { 255, 255, 255, 0 }
#define gateway \
  { 10, 0, 0, 1 }
#define dns \
  { 10, 0, 0, 1 }

  WiFi.mode(WIFI_AP_STA);

  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // We start by connecting to WiFi Station
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  delay(1000);

  //setting the static addresses in function "wifi_Start
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;

  WiFi.config(ip, gateway, subnet, dns);

  Serial.println("Web server running. Waiting for the ESP32 IP...");

  // Printing the ESP IP address
  Serial.print("Server IP:  ");
  Serial.println(WiFi.localIP());
  Serial.print("Port:  ");
  Serial.println("80");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  Serial.println("\n");

  delay(500);

  WiFi.waitForConnectResult();

  Serial.printf("Connection result: %d\n", WiFi.waitForConnectResult());

  server.begin();


  if (WiFi.waitForConnectResult() != 3) {
    delay(3000);
    wifi_Start();
  }
}
