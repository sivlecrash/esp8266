/* https://knxexpert.cz/mar/ */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
int status = WL_IDLE_STATUS;
const char* ssid = "******";
const char* pass = "******";
unsigned int localPort = 3671;
byte packetBuffer[30]; // buffer packets
byte packetBufferWrite[30]; // buffer packets
WiFiUDP Udp;
IPAddress ipMulti(224, 0, 23, 12);
unsigned int portMulti = 3671;
int loopNumber = 0;
//Individual Address - 1.1.1 Area [4 bit] . Line [4 bit] . Bus device [1 byte]
uint8_t KNXarea = 10;
uint8_t KNXline = 0;
uint8_t KNXdevice = 1;

int relayPinD0 = D0;
int relayPinD1 = D1;
int relayPinD2 = D2;
int relayPinD3 = D3;
int relayPinD4 = D4;
int relayPinD5 = D5;
int relayPinD6 = D6;
int relayPinD7 = D7;
int relayPinD8 = D8;
boolean relayPinD0state = 0;
boolean relayPinD1state = 0;
boolean relayPinD2state = 0;
boolean relayPinD3state = 0;
boolean relayPinD4state = 0;
boolean relayPinD5state = 0;
boolean relayPinD6state = 0;
boolean relayPinD7state = 0;
boolean relayPinD8state = 0;

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void setup() {
  pinMode(relayPinD0, OUTPUT);
  pinMode(relayPinD1, OUTPUT);
  pinMode(relayPinD2, OUTPUT);
  pinMode(relayPinD3, OUTPUT);
  pinMode(relayPinD4, OUTPUT);
  pinMode(relayPinD5, OUTPUT);
  pinMode(relayPinD6, OUTPUT);
  pinMode(relayPinD7, OUTPUT);
  pinMode(relayPinD8, OUTPUT);
  digitalWrite(relayPinD0, HIGH); // Off
  digitalWrite(relayPinD1, HIGH); // Off
  digitalWrite(relayPinD2, HIGH); // Off
  digitalWrite(relayPinD3, HIGH); // Off
  digitalWrite(relayPinD4, HIGH); // Off
  digitalWrite(relayPinD5, HIGH); // Off
  digitalWrite(relayPinD6, HIGH); // Off
  digitalWrite(relayPinD7, HIGH); // Off
  digitalWrite(relayPinD8, HIGH); // Off
  Serial.begin(115200);
  packetBufferWrite[0] = 6;     //ok  6[HEX]
  packetBufferWrite[1] = 16;    //ok 10[HEX] - Protocol version (constant) [1byte]
  packetBufferWrite[2] = 5;     //ok  5[HEX] - Service Type ID [2byte]
  packetBufferWrite[3] = 48;    //ok 30[HEX] - Service Type ID [2byte]
  packetBufferWrite[4] = 0;     //ok  0[HEX] - KNXnet/IP action [8bit+]
  packetBufferWrite[5] = 17;    //   11[HEX] KNXnet/IP action [+4bit], Total length [4bit]
  packetBufferWrite[6] = 41;    //ok 29[HEX] - Message Code [1byte]
  packetBufferWrite[7] = 0;     //ok  0[HEX] - Additional info [1byte...]
  packetBufferWrite[8] = 188;;  //ok BC[HEX] - Frame [1bit], Reserved [1bit], Repeat [1bit], Broadcast [1bit], Priority [2bit], ACK [1bit], error [1bit]
  packetBufferWrite[9] = 224;   //ok E0[HEX] - Type Destination [1bit], Routing [3bit], Ext. Frame Format [4bit]
  KNXarea <<= 4;
  packetBufferWrite[10] = KNXarea | KNXline; // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
  packetBufferWrite[11] = KNXdevice;         // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
  packetBufferWrite[12] = 0;    //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[13] = 1;    //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[14] = 1;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
  packetBufferWrite[15] = 0;    //ok 0[HEX] TPCI [6 bit], APCI(Type) [2 bit+]
  packetBufferWrite[16] = 0;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
  packetBufferWrite[17] = 0;    //Data
  packetBufferWrite[18] = 0;    //Data
  packetBufferWrite[19] = 0;    //Data
  packetBufferWrite[20] = 0;    //Data
  packetBufferWrite[21] = 0;    //Data
  packetBufferWrite[22] = 0;    //Data
  packetBufferWrite[23] = 0;    //Data
  packetBufferWrite[24] = 0;    //Data
  packetBufferWrite[25] = 0;    //Data
  packetBufferWrite[26] = 0;    //Data
  packetBufferWrite[27] = 0;    //Data
  packetBufferWrite[28] = 0;    //Data
  packetBufferWrite[29] = 0;    //Data
  packetBufferWrite[30] = 0;    //Data

  WiFi.begin(ssid, pass);
  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
  printWifiStatus();
  Serial.println("Connected to wifi");
  Serial.print("Udp Multicast server started at : ");
  Serial.print(ipMulti);
  Serial.print(":");
  Serial.println(portMulti);
  Udp.beginMulticast(WiFi.localIP(),  ipMulti, portMulti);
}

void loop()
{
  loopNumber++;
  if (loopNumber > 500) {
    loopNumber = 0;
  }
  int noBytes = Udp.parsePacket();

  if ( noBytes ) {
    Udp.read(packetBuffer, noBytes); // read the packet into the buffer
    //Protocol version [1byte]
    uint8_t protocol = packetBuffer[1];

    //Service Type ID [2byte]
    uint16_t stid = 0;
    stid = packetBuffer[2];
    stid <<= 8;
    stid = stid | packetBuffer[3];

    //KNXnet/IP action [12bit], Total length [4bit]
    uint16_t ipaction = 0;
    uint8_t ipaction2 = packetBuffer[5];
    uint8_t tlength = packetBuffer[5];
    ipaction2 >>= 4;
    ipaction2 <<= 4;
    ipaction = packetBuffer[4];
    ipaction <<= 8;
    ipaction = ipaction | ipaction2;
    tlength <<= 4;
    tlength >>= 4;

    //Message Code [1byte]
    uint8_t mcode = packetBuffer[6];

    //Additional info [1byte...]
    uint8_t ainfo = packetBuffer[7];

    //Control Field - Frame [1bit], Reserved [1bit], Repeat [1bit], Broadcast [1bit], Priority [2bit], ACK [1bit], error [1bit]
    uint8_t frame = packetBuffer[8];
    uint8_t reserved = packetBuffer[8];
    uint8_t repeat = packetBuffer[8];
    uint8_t broadcast = packetBuffer[8];
    uint8_t priority = packetBuffer[8];
    uint8_t ack = packetBuffer[8];
    uint8_t error = packetBuffer[8];
    frame >>= 7;
    reserved <<= 1;
    reserved >>= 7;
    repeat <<= 2;
    repeat >>= 7;
    broadcast <<= 3;
    broadcast >>= 7;
    priority <<= 4;
    priority >>= 6;
    ack <<= 6;
    ack >>= 7;
    error <<= 7;
    error >>= 7;

    //Control Field -  Type Destination [1bit], Routing [3bit], Ext. Frame Format [4bit]
    uint8_t destination = packetBuffer[9];
    uint8_t rout = packetBuffer[9];
    uint8_t efformat = packetBuffer[9];
    destination >>= 7;
    rout <<= 1;
    rout >>= 5;
    efformat <<= 4;
    efformat >>= 4;

    //Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
    uint8_t iaarea = packetBuffer[10];
    uint8_t ialine = packetBuffer[10];
    uint8_t iadevice = packetBuffer[11];
    iaarea >>= 4;
    ialine <<= 4;
    ialine >>= 4;

    //Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    uint8_t gamain = packetBuffer[12];
    uint8_t gamiddle = packetBuffer[12];
    uint8_t gasub = packetBuffer[13];
    gamain >>= 3;
    gamiddle <<= 5;
    gamiddle >>= 5;

    //AT [1 bit], NPCI [3 bit], Length [4 bit]
    uint8_t at = packetBuffer[14];
    uint8_t npci = packetBuffer[14];
    uint8_t length2 = packetBuffer[14];
    at >>= 7;
    npci <<= 1;
    npci >>= 5;
    length2 <<= 4;
    length2 >>= 4;

    //TPCI [6 bit], APCI(Type) [4 bit], DATA/APCI [6 bit]
    uint8_t tpci = packetBuffer[15];
    uint8_t apci = 0;
    uint8_t apci1 = packetBuffer[15];
    uint8_t apci2 = packetBuffer[16];
    uint8_t data16 = packetBuffer[16];
    tpci >>= 2;
    apci1 <<= 6;
    apci1 >>= 4;
    apci2 >>= 6;
    apci = apci1 | apci2;
    data16 <<= 2;
    data16 >>= 2;

    //Data 1byte - 14byte
    uint8_t data17 = packetBuffer[17];
    uint8_t data18 = packetBuffer[18];
    uint8_t data19 = packetBuffer[19];
    uint8_t data20 = packetBuffer[20];
    uint8_t data21 = packetBuffer[21];
    uint8_t data22 = packetBuffer[22];
    uint8_t data23 = packetBuffer[23];
    uint8_t data24 = packetBuffer[24];
    uint8_t data25 = packetBuffer[25];
    uint8_t data26 = packetBuffer[26];
    uint8_t data27 = packetBuffer[27];
    uint8_t data28 = packetBuffer[28];
    uint8_t data29 = packetBuffer[29];
    uint8_t data30 = packetBuffer[30];

    //--------------D0----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 0)) {  // if "1bit" + "Read" + "1/0/0"
      Serial.print("1/0/0 - relayPinD0 - Read - ");
      Serial.println(relayPinD0state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 0;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD0state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 0)) {  // if "1bit" + "Write" + "1/0/0"
      Serial.print("1/0/0 - relayPinD0 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD0, HIGH);
        relayPinD0state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD0, LOW);
        relayPinD0state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D1----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Read" + "1/0/1"
      Serial.print("1/0/1 - relayPinD1 - Read - ");
      Serial.println(relayPinD1state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 1;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD1state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Write" + "1/0/1"
      Serial.print("1/0/1 - relayPinD1 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD1, HIGH);
        relayPinD1state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD1, LOW);
        relayPinD1state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D2----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 2)) {  // if "1bit" + "Read" + "1/0/2"
      Serial.print("1/0/2 - relayPinD2 - Read - ");
      Serial.println(relayPinD2state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 2;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD2state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 2)) {  // if "1bit" + "Write" + "1/0/2"
      Serial.print("1/0/2 - relayPinD2 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD2, HIGH);
        relayPinD2state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD2, LOW);
        relayPinD2state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D3----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 3)) {  // if "1bit" + "Read" + "1/0/3"
      Serial.print("1/0/3 - relayPinD3 - Read - ");
      Serial.println(relayPinD3state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 3;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD3state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 3)) {  // if "1bit" + "Write" + "1/0/3"
      Serial.print("1/0/3 - relayPinD3 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD3, HIGH);
        relayPinD3state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD3, LOW);
        relayPinD3state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D4----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 4)) {  // if "1bit" + "Read" + "1/0/4"
      Serial.print("1/0/4 - relayPinD4 - Read - ");
      Serial.println(relayPinD4state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 4;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD4state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 4)) {  // if "1bit" + "Write" + "1/0/4"
      Serial.print("1/0/4 - relayPinD4 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD4, HIGH);
        relayPinD4state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD4, LOW);
        relayPinD4state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D5----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 5)) {  // if "1bit" + "Read" + "1/0/5"
      Serial.print("1/0/5 - relayPinD5 - Read - ");
      Serial.println(relayPinD5state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 5;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD5state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 5)) {  // if "1bit" + "Write" + "1/0/5"
      Serial.print("1/0/5 - relayPinD5 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD5, HIGH);
        relayPinD5state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD5, LOW);
        relayPinD5state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D6----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 6)) {  // if "1bit" + "Read" + "1/0/6"
      Serial.print("1/0/6 - relayPinD6 - Read - ");
      Serial.println(relayPinD6state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 6;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD6state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 6)) {  // if "1bit" + "Write" + "1/0/6"
      Serial.print("1/0/6 - relayPinD6 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD6, HIGH);
        relayPinD6state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD6, LOW);
        relayPinD6state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D7----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 7)) {  // if "1bit" + "Read" + "1/0/7"
      Serial.print("1/0/7 - relayPinD7 - Read - ");
      Serial.println(relayPinD7state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 7;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD7state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 7)) {  // if "1bit" + "Write" + "1/0/7"
      Serial.print("1/0/7 - relayPinD7 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD7, HIGH);
        relayPinD7state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD7, LOW);
        relayPinD7state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------D8----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 8)) {  // if "1bit" + "Read" + "1/0/8"
      Serial.print("1/0/8 - relayPinD8 - Read - ");
      Serial.println(relayPinD8state);
      uint8_t KNXipaction2 = 1;
      uint8_t KNXdpt = 1; // 1=1bit
      KNXipaction2 <<= 4;
      packetBufferWrite[5] = KNXipaction2 | KNXdpt;    // 11 KNXnet/IP action [+4bit], Total length [4bit]
      uint8_t KNXgaMain = 1;
      uint8_t KNXgaMiddle = 0;
      uint8_t KNXgaSub = 8;
      KNXgaMain <<= 3;
      packetBufferWrite[12] = KNXgaMain | KNXgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = KNXgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t KNXat = 0; //0[HEX]
      uint8_t KNXnpci = 0; //0[HEX]
      KNXat <<= 7;
      KNXnpci <<= 6;
      packetBufferWrite[14] = KNXat | KNXnpci | KNXdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t KNXapci2 = 1;
      KNXapci2 <<= 6;
      packetBufferWrite[16] = KNXapci2 | relayPinD8state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 8)) {  // if "1bit" + "Write" + "1/0/8"
      Serial.print("1/0/8 - relayPinD8 - ");
      if (data16 == 0) {
        digitalWrite(relayPinD8, HIGH);
        relayPinD8state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD8, LOW);
        relayPinD8state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------CENTRAL----------------//
    if  ((tlength == 1) && (apci == 2) && (gamain == 0) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Write" + "0/0/1"
      Serial.print("0/0/1 CENTRAL - ");
      if (data16 == 0) {
        digitalWrite(relayPinD0, HIGH);
        digitalWrite(relayPinD1, HIGH);
        digitalWrite(relayPinD2, HIGH);
        digitalWrite(relayPinD3, HIGH);
        digitalWrite(relayPinD4, HIGH);
        digitalWrite(relayPinD5, HIGH);
        digitalWrite(relayPinD6, HIGH);
        digitalWrite(relayPinD7, HIGH);
        digitalWrite(relayPinD8, HIGH);
        relayPinD0state = false;
        relayPinD1state = false;
        relayPinD2state = false;
        relayPinD3state = false;
        relayPinD4state = false;
        relayPinD5state = false;
        relayPinD6state = false;
        relayPinD7state = false;
        relayPinD8state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPinD0, LOW);
        digitalWrite(relayPinD1, LOW);
        digitalWrite(relayPinD2, LOW);
        digitalWrite(relayPinD3, LOW);
        digitalWrite(relayPinD4, LOW);
        digitalWrite(relayPinD5, LOW);
        digitalWrite(relayPinD6, LOW);
        digitalWrite(relayPinD7, LOW);
        digitalWrite(relayPinD8, LOW);
        relayPinD0state = true;
        relayPinD1state = true;
        relayPinD2state = true;
        relayPinD3state = true;
        relayPinD4state = true;
        relayPinD5state = true;
        relayPinD6state = true;
        relayPinD7state = true;
        relayPinD8state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //end
    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }
}
