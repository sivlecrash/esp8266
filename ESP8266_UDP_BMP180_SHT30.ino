/* https://knxexpert.cz/mar/ */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
int status = WL_IDLE_STATUS;
const char* ssid = "******";
const char* pass = "******";
unsigned int localPort = 3671;
byte packetBuffer[30];
byte packetBufferWrite[30];
WiFiUDP Udp;
IPAddress ipMulti(224, 0, 23, 12);
unsigned int portMulti = 3671;
int loopNumber = 0;
long D1miniTime = 0;
long nextTime = 0;
long millisecondsConstant = 0;
int minutesConstant = 3; // periodic sending, time + device number


//BME180
#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
float UDPvalueT = 0;
float calibrationT = -4.7;
float UDPvalueP = 0;

//SHT30
#include <WEMOS_SHT3X.h>
SHT3X sht30(0x45);
int UDPvalueH = 0;

//Individual Address - 1.1.1 Area [4 bit] . Line [4 bit] . Bus device [1 byte]
uint8_t UDParea = 10;
uint8_t UDPline = 0;
uint8_t UDPdevice = 254;

//UDPsend
uint8_t UDPipaction2 = 0;
uint8_t UDPdpt = 0;
uint8_t UDPgaMain = 0;
uint8_t UDPgaMiddle = 0;
uint8_t UDPgaSub = 0;
uint8_t UDPat = 0;
uint8_t UDPnpci = 0;
uint8_t F01a = 0;
uint8_t F01b = 0;
uint16_t F01c = 0;
uint32_t F01cm = 0;
uint16_t F01c1 = 0;
uint16_t F01c2 = 0;
uint8_t UDPvalueH5 = 0;
uint8_t F02a = 0;
uint8_t F02b = 0;
uint16_t F02c = 0;
uint32_t F02cm = 0;
uint16_t F02c1 = 0;
uint16_t F02c2 = 0;

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void setup() {
  Serial.begin(115200);

  //BME280
  if (!bmp.begin())
  {
    Serial.println("Could not find BMP180 or BMP085 sensor at 0x77");
    while (1) {}
  }

  //SHT30
  sht30.get();

  //UDPsend
  millisecondsConstant = ((UDParea + UDPline + UDPdevice) * 1000) + (minutesConstant * 60000);
  nextTime = millisecondsConstant;
  packetBufferWrite[0] = 6;     //ok  6[HEX]
  packetBufferWrite[1] = 16;    //ok 10[HEX] - Protocol version (constant) [1byte]
  packetBufferWrite[2] = 5;     //ok  5[HEX] - Service Type ID [2byte]
  packetBufferWrite[3] = 48;    //ok 30[HEX] - Service Type ID [2byte]
  packetBufferWrite[4] = 0;     //ok  0[HEX] - UDPnet/IP action [8bit+]
  packetBufferWrite[5] = 17;    //   11[HEX] UDPnet/IP action [+4bit], Total length [4bit]
  packetBufferWrite[6] = 41;    //ok 29[HEX] - Message Code [1byte]
  packetBufferWrite[7] = 0;     //ok  0[HEX] - Additional info [1byte...]
  packetBufferWrite[8] = 188;;  //ok BC[HEX] - Frame [1bit], Reserved [1bit], Repeat [1bit], Broadcast [1bit], Priority [2bit], ACK [1bit], error [1bit]
  packetBufferWrite[9] = 224;   //ok E0[HEX] - Type Destination [1bit], Routing [3bit], Ext. Frame Format [4bit]
  UDParea <<= 4;
  packetBufferWrite[10] = UDParea | UDPline; // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
  packetBufferWrite[11] = UDPdevice;         // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
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

    //UDPnet/IP action [12bit], Total length [4bit]
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

    //--------------BMP180-Temperature °C 2Byte DPT9.001---------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 4) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Read" + "4/0/1"
      Serial.print("Response - 4/0/1 - BME180 - Temperature: ");
      UDPipaction2 = 1;
      UDPdpt = 3; // 1=1bit 2=1Byte 3=2Byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 4;
      UDPgaMiddle = 0;
      UDPgaSub = 1;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      UDPat = 0; //0[HEX]
      UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      packetBufferWrite[16] = 64;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
      UDPvalueT = (bmp.readTemperature() + calibrationT);
      F01a = 0; // [1bit]
      F01b = 0; // [4bit]
      F01c = 0; // [11bit]
      if (UDPvalueT < 0) { // minus
        F01a = 1;
        F01cm = ((UDPvalueT / pow(2, F01b)) / 0.01);
        while (F01cm < -2048) {
          F01b++;
          F01cm = ((UDPvalueT / pow(2, F01b)) / 0.01);
        }
        F01c = F01cm + 2048;
      }

      else if (UDPvalueT > 0) { // plus
        F01a = 0;
        F01c = (UDPvalueT / pow(2, F01b)) / 0.01;
        while (F01c > 2047) {
          F01b++;
          F01c = (UDPvalueT / pow(2, F01b)) / 0.01;
        }
      }
      else {
        F01a = 0;
        F01b = 0;
        F01c = 0;
      }
      F01a <<= 7;
      F01b <<= 3;
      F01c1 = F01c;
      F01c1 <<= 5;
      F01c1 >>= 13;
      F01c2 = F01c;
      F01c2 <<= 8;
      F01c2 >>= 8;
      packetBufferWrite[17] = F01a | F01b | F01c1;
      packetBufferWrite[18] = F01c2;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 19);
      Udp.endPacket();
      Serial.print(UDPvalueT);
      Serial.println(" C ; ");
    }

    //--------------SHT30-hum-0...100% 1Byte DPT5.001--------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 5) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Read" + "5/0/1"
      Serial.print("Response - 5/0/1 - SHT30 - Relative humidity: ");
      UDPipaction2 = 1;
      UDPdpt = 2; // 1=1bit 2=1byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 5;
      UDPgaMiddle = 0;
      UDPgaSub = 1;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      UDPat = 0; //0[HEX]
      UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      packetBufferWrite[16] = 64;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
      sht30.get();
      UDPvalueH = sht30.humidity;
      UDPvalueH5 = UDPvalueH * 2.55;
      packetBufferWrite[17] = UDPvalueH5; // 0...100 to 0...255 // Percent = (x * 100 + 128) / 256
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 18);
      Udp.endPacket();
      Serial.print(UDPvalueH);
      Serial.println(" %");
    }

    //--------------BMP180-Pressure Pa 2Byte DPT9.006---------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 5) && (gamiddle == 1) && (gasub == 1)) {  // if "1bit" + "Read" + "5/1/1"
      Serial.print("Response - 5/1/1 - BMP180 - Pressure: ");
      UDPipaction2 = 1;
      UDPdpt = 3; // 1=1bit 2=1Byte 3=2Byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 5;
      UDPgaMiddle = 1;
      UDPgaSub = 1;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      UDPat = 0; //0[HEX]
      UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      packetBufferWrite[16] = 64;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
      UDPvalueP = (bmp.readPressure() / 100.0F);
      F02a = 0; // [1bit]
      F02b = 0; // [4bit]
      F02c = 0; // [11bit]
      if (UDPvalueP < 0) { // minus
        F02a = 1;
        F02cm = ((UDPvalueP / pow(2, F02b)) / 0.01);
        while (F02cm < -2048) {
          F02b++;
          F02cm = ((UDPvalueP / pow(2, F02b)) / 0.01);
        }
        F02c = F02cm + 2048;
      }
      else if (UDPvalueP > 0) { // plus
        F02a = 0;
        F02c = (UDPvalueP / pow(2, F02b)) / 0.01;
        while (F02c > 2047) {
          F02b++;
          F02c = (UDPvalueP / pow(2, F02b)) / 0.01;
        }
      }
      else {
        Serial.print(" //000: ");
        F02a = 0;
        F02b = 0;
        F02c = 0;
      }
      F02a <<= 7;
      F02b <<= 3;
      F02c1 = F02c;
      F02c1 <<= 5;
      F02c1 >>= 13;
      F02c2 = F02c;
      F02c2 <<= 8;
      F02c2 >>= 8;
      packetBufferWrite[17] = F02a | F02b | F02c1;
      packetBufferWrite[18] = F02c2;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 19);
      Udp.endPacket();
      Serial.print(UDPvalueP);
      Serial.println(" hPa ");
    }

    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }

  D1miniTime = millis();

  if (D1miniTime > nextTime) {
    nextTime = D1miniTime + millisecondsConstant;
    UDPvalueT = (bmp.readTemperature() + calibrationT);
    sht30.get();
    UDPvalueH = sht30.humidity;
    UDPvalueP = (bmp.readPressure() / 100.0F);

    //--------------BMP180-Temperature: 2Byte DPT9.001---------------//
    Serial.print("Write - 4/0/1 - BME280 - Temperature: ");
    UDPipaction2 = 1;
    UDPdpt = 3; // 1=1bit 2=1Byte 3=2Byte
    UDPipaction2 <<= 4;
    packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
    UDPgaMain = 4;
    UDPgaMiddle = 0;
    UDPgaSub = 1;
    UDPgaMain <<= 3;
    packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    UDPat = 0; //0[HEX]
    UDPnpci = 0; //0[HEX]
    UDPat <<= 7;
    UDPnpci <<= 6;
    packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
    packetBufferWrite[16] = 128;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
    F01a = 0; // [1bit]
    F01b = 0; // [4bit]
    F01c = 0; // [11bit]
    if (UDPvalueT < 0) { // minus
      F01a = 1;
      F01cm = ((UDPvalueT / pow(2, F01b)) / 0.01);
      while (F01cm < -2048) {
        F01b++;
        F01cm = ((UDPvalueT / pow(2, F01b)) / 0.01);
      }
      F01c = F01cm + 2048;
    }

    else if (UDPvalueT > 0) { // plus
      F01a = 0;
      F01c = (UDPvalueT / pow(2, F01b)) / 0.01;
      while (F01c > 2047) {
        F01b++;
        F01c = (UDPvalueT / pow(2, F01b)) / 0.01;
      }
    }
    else {
      F01a = 0;
      F01b = 0;
      F01c = 0;
    }
    F01a <<= 7;
    F01b <<= 3;
    F01c1 = F01c;
    F01c1 <<= 5;
    F01c1 >>= 13;
    F01c2 = F01c;
    F01c2 <<= 8;
    F01c2 >>= 8;
    packetBufferWrite[17] = F01a | F01b | F01c1;
    packetBufferWrite[18] = F01c2;
    Udp.beginPacket(ipMulti, portMulti);
    Udp.write(packetBufferWrite, 19);
    Udp.endPacket();
    Serial.print(UDPvalueT);
    Serial.println(" C ; ");
    delay(5);
    //--------------SHT30-hum-0...100% 1Byte DPT5.001--------------//
    Serial.print("Write - 5/0/1 - SHT30 - Relative humidity: ");
    UDPipaction2 = 1;
    UDPdpt = 2; // 1=1bit 2=1byte
    UDPipaction2 <<= 4;
    packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
    UDPgaMain = 5;
    UDPgaMiddle = 0;
    UDPgaSub = 1;
    UDPgaMain <<= 3;
    packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    UDPat = 0; //0[HEX]
    UDPnpci = 0; //0[HEX]
    UDPat <<= 7;
    UDPnpci <<= 6;
    packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
    packetBufferWrite[16] = 128;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
    UDPvalueH5 = UDPvalueH * 2.55;
    packetBufferWrite[17] = UDPvalueH5; // 0...100 to 0...255 // Percent = (x * 100 + 128) / 256
    Udp.beginPacket(ipMulti, portMulti);
    Udp.write(packetBufferWrite, 18);
    Udp.endPacket();
    Serial.print(UDPvalueH);
    Serial.println(" %");
    delay(5);

    //--------------BMP180-Pressure Pa 2Byte DPT9.006---------------//
    Serial.print("Write - 5/1/1 - BME280 - Pressure: ");
    UDPipaction2 = 1;
    UDPdpt = 3; // 1=1bit 2=1Byte 3=2Byte
    UDPipaction2 <<= 4;
    packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
    UDPgaMain = 5;
    UDPgaMiddle = 1;
    UDPgaSub = 1;
    UDPgaMain <<= 3;
    packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    UDPat = 0; //0[HEX]
    UDPnpci = 0; //0[HEX]
    UDPat <<= 7;
    UDPnpci <<= 6;
    packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
    packetBufferWrite[16] = 128;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
    F02a = 0; // [1bit]
    F02b = 0; // [4bit]
    F02c = 0; // [11bit]
    if (UDPvalueP < 0) { // minus
      F02a = 1;
      F02cm = ((UDPvalueP / pow(2, F02b)) / 0.01);
      while (F02cm < -2048) {
        F02b++;
        F02cm = ((UDPvalueP / pow(2, F02b)) / 0.01);
      }
      F02c = F02cm + 2048;
    }
    else if (UDPvalueP > 0) { // plus
      F02a = 0;
      F02c = (UDPvalueP / pow(2, F02b)) / 0.01;
      while (F02c > 2047) {
        F02b++;
        F02c = (UDPvalueP / pow(2, F02b)) / 0.01;
      }
    }
    else {
      Serial.print(" //000: ");
      F02a = 0;
      F02b = 0;
      F02c = 0;
    }
    F02a <<= 7;
    F02b <<= 3;
    F02c1 = F02c;
    F02c1 <<= 5;
    F02c1 >>= 13;
    F02c2 = F02c;
    F02c2 <<= 8;
    F02c2 >>= 8;
    packetBufferWrite[17] = F02a | F02b | F02c1;
    packetBufferWrite[18] = F02c2;
    Udp.beginPacket(ipMulti, portMulti);
    Udp.write(packetBufferWrite, 19);
    Udp.endPacket();
    Serial.print(UDPvalueP);
    Serial.println(" hPa ");
  }
  // }
}
