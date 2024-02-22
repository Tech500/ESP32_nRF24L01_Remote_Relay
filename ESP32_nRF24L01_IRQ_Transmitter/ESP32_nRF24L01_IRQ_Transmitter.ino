//  ESP32_nRF24L01_IRQ_Transmitter.ino  William Lucid 02/19/2024 @ 21:34 EST
//  Based on RF24 library example "InterruptConfigure"
//  https://github.com/nRF24/RF24/blob/update-irq-example/examples/InterruptConfigure/InterruptConfigure.ino
//  Example:  Author: Brendan Doherty (2bndy5)
//  Addition code, references, guidance, and lots of help:  Google's Bard.

//  Path:  "M:\■■■ Latest nRF24L01 Project\ESP32_nRF24L01_IRQ_Transmitter\ESP32_nRF24L01_IRQ_Transmitter.ino"

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
#define SCK  18
#define SS    5
#define CSN  21

#define IRQ_PIN 4

// Replace with your network details
const char *ssid = "xxxx";
const char *password = "xxxxxxx";

WiFiClient client;

AsyncWebServer server(80);

int data = 1;

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
char tx_payloads[][tx_pl_size + 1] = { "Ping ", "Pong ", "Radio", "1FAIL" };
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
  batteryOff();
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
  radio.setAutoAck(false);
  radio.setChannel(7);
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

  radio.stopListening();  //This sets the module as transmitter

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, PSTR("text/html"), HTML7, processor7);
    data = 1;
    needAnotherCountdown = 1;
    countdownTrigger();
  });

  server.begin();

  oneTick.attach(1.0, ISRwatchdog);  //watchdog  ISR triggers every 1 second
}
 

void loop() {}

String processor7(const String &var) {

  //index7.h

  if (var == F("LINK"))
    return linkAddress;

  return String();
}

void batteryOff() {
  int data = 2;
  switchOne(data);
  oneTick.detach();
}

void countdownTrigger() {
  // Perform countdown actions here
  Serial.println("\nCountdown timer triggered!\n");
  // Schedule the next countdown if needed
  if (needAnotherCountdown == 1) {
    onceTick.once(60, ISRcamera);
    int data = 1;
    switchOne(data);
    needAnotherCountdown = 0;
  }
}

void switchOne(int data) {

  if (data == 1) {
    data = 1;
    payload.switchState = data;
    Serial.println("Battery Switch is ON");
    Serial.println("ESP32 waking from Deep Sleep\n");
  }

  if (data == 2) {
    data = 2;
    payload.switchState = data;
    Serial.println("Battery power switched OFF");
    Serial.println("ESP32 going to Deep Sleep\n");
  }

  radio.write(&payload, sizeof(payload));  //Sending data value

  if(! radio.write(&payload, sizeof(payload))){
    Serial.println("\nTransmit failed!!!");
  }

  radio.flush_tx();
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
