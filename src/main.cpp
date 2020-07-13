/*
Original code created January 4th, 2014 by Claude Heintz http://lx.claudeheintzdesign.com/lxarduino_sketches.html
modified by: Jared Alexander, Mike Skutta

This code is in the public domain.
sACN E 1.31 is a public standard published by the PLASA technical standards program
http://tsp.plasa.org/tsp/documents/published_docs.php

Requires WEMOS D1 Mini with the Relay Shield 
Receives E1.31 data (SACN) and ArtNet data for control of relays. 
You can only send the Arduino one protocol (E1.31 or ArtNet) at a time. 
*/

#if !(defined(ESP_NAME))
  #define ESP_NAME "relay"
#endif

#if !(defined(CHANNEL))
  #define CHANNEL 1 // Channel is 1 based
#endif
/*  set the desired subnet and universe (first universe is 0)
    sACN starts with universe 1 so subtract 1 from sACN universe
    to get DMX_UNIVERSE.                                           */
#if !(defined(ARTNET_SUBNET))
  #define ARTNET_SUBNET 0 //defualt subnet is 0. Should not need to be changed. 
#endif
#if !(defined(ARTNET_UNIVERSE))
  #define ARTNET_UNIVERSE 0 //first universe being used
#endif
#if !(defined(E131_SUBNET))
  #define E131_SUBNET 0 //default subnet is 0. Should not need to be changed.
#endif 
#define ETHERNET_BUFFER_MAX 640
#define ARTNET_ARTDMX 0x5000
#define ARTNET_ARTPOLL 0x2000
#define ARTNET_PORT 0x1936 //standard port
#define ARTNET_START_ADDRESS 18 + CHANNEL -1 //Byte 18 in the packet contains channel 1 values. 
#define E131_PORT 5568 //standard port 
#define E131_START_ADDRESS 126 + CHANNEL -1 //Byte 126 in the packet contains channel 1 values. Offset is set to one prior. 
#define NUM_RELAYS 1 //total number of relays used

#include <Arduino.h>

#include <ESP8266WiFi.h> // WIFI support
#include <ESP8266mDNS.h> // For network discovery
#include <WiFiUdp.h> // UDP
#include <ArduinoOTA.h> // Updates over the air

// WiFi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 

int channel; //channel increment 

//Relay Pins array
int relay[] = {D1, D2, D3, D4, D5, D6, D7, D8,};

//Timer Setup
volatile byte currentcounter = 0; //counter for data reception
byte previouscounter = 0; //counter for data reception 
unsigned long currentDelay = 0; //current time value for ArtNet and E1.31 reception

// buffer to hold E1.31 and Artnet data
unsigned char packetBuffer[ETHERNET_BUFFER_MAX];

// LED
#include <Ticker.h>
Ticker ticker;
int ledState = LOW;
unsigned long ledNextRun = 0;

/* WIFI */
char hostname[32] = {0};

/* UDP */
WiFiUDP ArtNetUdp;
WiFiUDP E131Udp;

void tick()
{
  //toggle state
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println(F("Config Mode"));
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  ticker.attach(0.2, tick);
}

void setup() {
  /* Serial and I2C */
  Serial.begin(9600);

  /* LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize Relay Pins */
  for(int a = 0; a < NUM_RELAYS; a++) //loop to initialize relay control pins 
  {
   pinMode(relay[a], OUTPUT); //initialize relay output pins
   digitalWrite(relay[a], LOW); //set pins to high for off
  }

  /* Function Select */
  Serial.println(ESP_NAME);
  
  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if(!wifiManager.autoConnect(hostname)) {
    Serial.println("WiFi Connect Failed");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  /* UDP */
  ArtNetUdp.begin(ARTNET_PORT); //Open Artnet Port
  E131Udp.begin(E131_PORT); //Open E1.31 Port

  Serial.println(hostname);
  Serial.print(F("  "));
  Serial.println(WiFi.localIP());
  Serial.print(F("  "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("Art-Net:"));
  Serial.println(ArtNetUdp.localPort());
  Serial.print(F("E1.31:"));
  Serial.println(E131Udp.localPort());

  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

/* artDMXReceived checks the universe and subnet then
   outputs the data to the relays */
void artDMXReceived(unsigned char* pbuff) {
  if ( (pbuff[14] & 0xF) == ARTNET_UNIVERSE ) {
    if ( (pbuff[14] >> 8) == ARTNET_SUBNET ) {
      channel = 0; //reset channel offset to 0 each time through loop
      //loop turns relay on/off based on channel value starting at Artnet start address
      for(int b = 0; b < NUM_RELAYS; b++) {
        if(pbuff[ARTNET_START_ADDRESS + channel] > 127) { //if channel value is greater then 127
          digitalWrite(relay[b], HIGH); //turn relay on 
        } else {
          digitalWrite(relay[b], LOW); //else turn it off 
        }
        channel++; //increase channel offset by 1 each time through for loop
      }
    }
  }
} //end artDMXReceived

/*  artNetOpCode checks to see that the packet is actually Art-Net
    and returns the opcode telling what kind of Art-Net message it is.  */
int artNetOpCode(unsigned char* pbuff) {
  String test = String((char*)pbuff);
  if ( test.equals("Art-Net") ) {
    if ( pbuff[11] >= 14 ) { //protocol version [10] hi byte [11] lo byte
      return pbuff[9] *256 + pbuff[8];  //opcode lo byte first
    }
  }
  return 0;
}

/* sacnDMXReceived checks the universe and subnet then
   outputs the data to relays  */
void sacnDMXReceived(unsigned char* pbuff, int count) {
  if ( pbuff[113] == E131_SUBNET ) { 
    int addressOffset = 125; //first 125 bytes of packet are header information
    if ( pbuff[addressOffset] == 0 ) { //start code must be 0
      channel = 0; //reset channel offset to 0 each time through loop
      //loop turns relay on/off based on channel value starting at E1.31 start address
      for(int c = 0; c < NUM_RELAYS; c++) {
        if(pbuff[E131_START_ADDRESS + channel] > 127) { //if channel value is greater then 127
          digitalWrite(relay[c], LOW); //turn relay on
        } else {
          digitalWrite(relay[c], HIGH); //turn relay off 
        }
        channel++; //increment channel offset by 1
      }
    }
  } 
} //end sacnDMXReceived

//checks to see if packet is E1.31 data
int checkACNHeaders(unsigned char* messagein, int messagelength) {
  if ( messagein[1] == 0x10 && messagein[4] == 0x41 && messagein[12] == 0x37) { 
    int addresscount = messagein[123] * 256 + messagein[124]; // number of values plus start code
    return addresscount -1; //Return how many values are in the packet.
  }
  return 0;
}

/************************************************************************
  The main loop checks for and reads packets from the two UDP ethernet
  socket connections.  When a packet is recieved, it is checked to see if
  it is valid and then one of the DMXReceived functions is called, sending
  the DMX values to the output. There also is a timer to run a standby 
  program if no data is received for 30 seconds. 
*************************************************************************/
void loop() {
  ArduinoOTA.handle();

  if(currentcounter != previouscounter) { //has the value changed?
    currentDelay = millis(); //store the time since the value has increased 
    previouscounter = currentcounter; //set the previous value equal to the current value
  }
 
  if(millis() - currentDelay > 30000) { //is the time since the value changed greater than 30 seconds?
    digitalWrite(LED_BUILTIN, LOW); //turn LED off. Not receiving E1.31 or ArtNet. 
  }
 
  // first check to see if a packet is available on the Art-Net port
  int packetSize = ArtNetUdp.parsePacket();
  if( packetSize ) {
    ArtNetUdp.read(packetBuffer, ETHERNET_BUFFER_MAX);
    /* after reading the packet into the buffer, check to make sure
       that it is an Art-Net packet and retrieve the opcode that
       tells what kind of message it is                         */
    int opcode = artNetOpCode(packetBuffer);
    if ( opcode == ARTNET_ARTDMX ) {
      Serial.print("ArtNet Packet Received: ");
      Serial.println(currentcounter);
      artDMXReceived(packetBuffer);
      currentcounter++;  //increase counter by 1 each time through 
      digitalWrite(LED_BUILTIN, HIGH); //turn status LED on
    } 
  } else {
    /* then, if still no packet, check to see if a packet
        is available on the sACN unicastcast port         */
    packetSize = E131Udp.parsePacket();
    if( packetSize ) {
      E131Udp.read(packetBuffer, ETHERNET_BUFFER_MAX);
      /* after reading the packet into the buffer, check to make sure
       that it is a valid sACN packet.*/
      int count = checkACNHeaders(packetBuffer, packetSize);
      if ( count ) {
        Serial.print("E131 Packet Received: ");
        Serial.println(currentcounter);
        sacnDMXReceived(packetBuffer, count);
        currentcounter++;  //increase counter by 1 each time through 
        digitalWrite(LED_BUILTIN, HIGH); //turn status LED on
      }
    }
  }
}