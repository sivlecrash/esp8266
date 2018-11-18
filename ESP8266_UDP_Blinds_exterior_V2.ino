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
long nextTime = 0;
long millisecondsConstant = 0;
int minutesConstant = 4; // periodic sending, time + device number

//Individual Address - 1.1.1 Area [4 bit] . Line [4 bit] . Bus device [1 byte]
uint8_t UDParea = 10;
uint8_t UDPline = 0;
uint8_t UDPdevice = 252;

//UDP
uint8_t UDPipaction2 = 0;
uint8_t UDPdpt = 0;
uint8_t UDPgaMain = 0;
uint8_t UDPgaMiddle = 0;
uint8_t UDPgaSub = 0;
uint8_t UDPat = 0;
uint8_t UDPnpci = 0;
uint32_t data4b17 = 0;
uint32_t data4b18 = 0;
uint32_t data4b19 = 0;
uint32_t data4b20 = 0;

//Blinds
const int Relay1Pin = D5; //! RelayShield - change D1 to D5
const int Relay2Pin = D6; //! RelayShield - change D1 to D6
boolean Relay1PinState;
boolean Relay2PinState;
// Relay1-on Relay2-off = blinds down
// Relay1-on Relay2-on = blinds up
// Relay1-off Relay2-off = blinds stop

long timeHeight = 38000; // [4Byte 0...4294967295ms] turn the blind down
long timeSlat = 2000; // [4Byte 0...4294967295ms] time of transition of the lamels
long timeStep = timeSlat / 5;

float timeSet = 0; // calculated engine time
long timeStatus;

long timeHeightSet;
long timeHeightStatus;
long timeSlatSet;
long timeSlatStatus;

char HeightSet; // [1Byte 0...100%] set
char SlatSet; // [1Byte 0...100%] l set
long HeightStatus = 0; // [1Byte 0...100%] last set
long SlatStatus = 0; // [1Byte 0...100%] last set

boolean BlindsDirection; // the last direction of the blinds
long D1miniTime = 0; // = millis()
long D1miniTimeStart = 0; // millis() - engine start time
long D1miniTimeStop = 0; // millis() - engine stoppage time
long D1miniTimeSetStopMax = 0; // max millis() = STOP

float constant1byte = 2.55;

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void setup() {
  Serial.begin(115200);

  //Blinds
  pinMode(Relay1Pin, OUTPUT);
  pinMode(Relay2Pin, OUTPUT);
  digitalWrite(Relay1Pin, LOW);
  digitalWrite(Relay2Pin, LOW);

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

    //--------------WRITE-MOVE.CENTRAL---------------//

    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 6) && (gasub == 0)) {  // if "1bit" + "Write" + "3/6/0"
      if (data16 == 0) {
        Serial.println("3/0/0 Move - 0 - up - CENTRAL ");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeHeight + timeSlat;
          BlindsUP();
        }
        else if  ((Relay1PinState == 1) || (Relay1PinState == 1)) {
          BlindsSTOP();
        }
      }
      else if (data16 == 1) {
        Serial.println("3/0/0 Move - 1 - down");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeHeight;
          BlindsDOWN();
        }
        else if  ((Relay1PinState == 1) || (Relay1PinState == 1)) {
          BlindsSTOP();
        }
      }
    }

    //--------------WRITE-STEP.CENTRAL---------------//

    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 7) && (gasub == 0)) {  // if "1bit" + "Write" + "3/7/0"
      if (data16 == 0) {
        Serial.println("3/7/0 Move - 0 - up - CENTRAL ");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeStep;
          BlindsUP();
        }
        else {
          BlindsSTOP();
        }
      }
      else if (data16 == 1) {
        Serial.println("3/0/0 Move - 1 - down");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeStep;
          BlindsDOWN();
        }
        else {
          BlindsSTOP();
        }
      }
    }

    //--------------READ-Height---------------// -ok
    if  ((tlength == 1) && (apci == 0) && (gamain == 3) && (gamiddle == 4) && (gasub == 0)) {  // if "1bit" + "Read" + "3/4/0" - HeightStatus
      Serial.print("Response - 3/4/0 - HeightStatus - ");
      Serial.print(HeightStatus);
      Serial.print(" % = ");
      Serial.print(timeHeightStatus);
      Serial.print(" ms ");
      Serial.print(" slat status: ");
      Serial.print(SlatStatus);
      Serial.println(" %");
      UDPipaction2 = 1;
      UDPdpt = 2; // 1=1bit 2=1Byte 3=2Byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 3;
      UDPgaMiddle = 4;
      UDPgaSub = 0;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      UDPat = 0; //0[HEX]
      UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      packetBufferWrite[16] = 64;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
      packetBufferWrite[17] = HeightStatus * constant1byte;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 18);
      Udp.endPacket();
    }

    //--------------READ-Slat---------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 3) && (gamiddle == 5) && (gasub == 0)) {  // if "1bit" + "Read" + "3/5/0" - SlatStatus
      Serial.print("Response - 3/5/0 - SlatStatus - ");
      Serial.print(HeightStatus);
      Serial.print(" % = ");
      Serial.print(timeHeightStatus);
      Serial.print(" ms ");
      Serial.print(" slat status: ");
      Serial.print(SlatStatus);
      Serial.println(" %");
      UDPipaction2 = 1;
      UDPdpt = 2; // 1=1bit 2=1Byte 3=2Byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 3;
      UDPgaMiddle = 5;
      UDPgaSub = 0;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      UDPat = 0; //0[HEX]
      UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      packetBufferWrite[16] = 64;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
      packetBufferWrite[17] = SlatStatus * constant1byte;;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 18);
      Udp.endPacket();
    }

    //--------------WRITE-MOVE---------------//

    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 0) && (gasub == 0)) {  // if "1bit" + "Write" + "3/0/0"
      if (data16 == 0) {
        Serial.println("3/0/0 Move - 0 - up - CENTRAL ");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeHeight + timeSlat;
          BlindsUP();
        }
        else if  ((Relay1PinState == 1) || (Relay1PinState == 1)) {
          BlindsSTOP();
        }
      }
      else if (data16 == 1) {
        Serial.println("3/0/0 Move - 1 - down");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeHeight;
          BlindsDOWN();
        }
        else if  ((Relay1PinState == 1) || (Relay1PinState == 1)) {
          BlindsSTOP();
        }
      }
    }
    
    //--------------WRITE-STEP---------------//

    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 1) && (gasub == 0)) {  // if "1bit" + "Write" + "3/1/0"
      if (data16 == 0) {
        Serial.println("3/0/0 Move - 0 - up");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeStep;
          BlindsUP();
        }
        else {
          BlindsSTOP();
        }
      }
      else if (data16 == 1) {
        Serial.println("3/0/0 Move - 1 - down");
        if ((Relay1PinState == 0) && (Relay1PinState == 0)) {
          timeSet = timeStep;
          BlindsDOWN();
        }
        else {
          BlindsSTOP();
        }
      }
    }

    //--------------WRITE-HEIGHT-0...100---------------//
    if  ((tlength == 2) && (apci == 2) && (gamain == 3) && (gamiddle == 2) && (gasub == 0)) {  // if "1byte" + "Write" + "3/2/0"
      HeightSet = data17 / constant1byte;
      timeHeightSet = (timeHeight / 100) * HeightSet;
      timeSet = timeHeightSet - timeHeightStatus;
      Serial.print("3/2/0 - HeightSet: ");
      Serial.print(HeightSet);
      Serial.print(" timeHeightSet: ");
      Serial.print(timeHeightSet);
      Serial.print(" timeSet: ");
      Serial.println(timeSet);
      if (timeSet > 0) {
        BlindsDOWN();
      }
      else if (timeSet < 0) {
        timeSet = timeSet * -1;
        BlindsUP();
      }
      else  {
        BlindsStatusSending();
      }
    }

    //--------------WRITE-SLAT-0...100---------------//
    if  ((tlength == 2) && (apci == 2) && (gamain == 3) && (gamiddle == 3) && (gasub == 0)) {  // if "1byte" + "Write" + "3/3/0"
      SlatSet = data17 / constant1byte;
      timeSlatSet = (timeSlat / 100) * SlatSet;
      timeSet = timeSlatSet - timeSlatStatus;
      Serial.print("3/3/0 - SlatSet: ");
      Serial.print(SlatSet);
      Serial.print(" timeSlatSet: ");
      Serial.print(timeSlatSet);
      Serial.print(" timeSet: ");
      Serial.println(timeSet);
      if (timeSet > 0) {
        BlindsDOWN();
      }
      else if (timeSet < 0) {
        timeSet = timeSet * -1;
        BlindsUP();
      }
      else  {
        BlindsStatusSending();
      }
    }
    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }
  D1miniTime = millis();
  /*Serial.print(D1miniTime);
    Serial.println(" ,");*/
  if ((D1miniTimeSetStopMax == D1miniTime) && ((Relay1PinState == 1) || (Relay1PinState == 1)))  {
    BlindsSTOP();
    Serial.println("D1miniTimeSetStopMax - STOP");
  }
  if ((4294967295 == D1miniTime) && ((Relay1PinState == 1) || (Relay1PinState == 1))) {
    digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
    Relay1PinState = 0; Relay2PinState = 0;
    D1miniTimeStop = 4294967295;
    Serial.println("4294967295 - STOP");
  }
}

void BlindsUP() {
  digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
  Relay1PinState = 1; Relay2PinState = 1;
  BlindsDirection = 0; // 0 = up // 1 = down
  D1miniTime = millis();
  Serial.print(" D1miniTime: ");
  Serial.print(D1miniTime);
  D1miniTimeStart = millis();
  Serial.print(" D1miniTimeStart: ");
  Serial.print(D1miniTimeStart);
  D1miniTimeSetStopMax = D1miniTimeStart + timeSet;
  Serial.print(" D1miniTimeSetStopMax: ");
  Serial.print(D1miniTimeSetStopMax);
  Serial.println(" BlindsUP ");
}

void BlindsDOWN() {
  digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
  Relay1PinState = 1; Relay2PinState = 0;
  BlindsDirection = 1; // 0 = up // 1 = down
  D1miniTime = millis();
  Serial.print(" D1miniTime: ");
  Serial.print(D1miniTime);
  D1miniTimeStart = millis();
  Serial.print(" D1miniTimeStart: ");
  Serial.print(D1miniTimeStart);
  D1miniTimeSetStopMax = D1miniTimeStart + timeSet;
  Serial.print(" D1miniTimeSetStopMax: ");
  Serial.print(D1miniTimeSetStopMax);
  Serial.println(" BlindsDOWN ");
}

void BlindsSTOP() {
  digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
  Relay1PinState = 0; Relay2PinState = 0;
  D1miniTimeStop = millis();
  D1miniTimeSetStopMax = D1miniTimeStop;
  Serial.print(" D1miniTime: ");
  Serial.print(D1miniTime);
  Serial.print(" D1miniTimeStop: ");
  Serial.print(D1miniTimeStop);
  Serial.println(" BlindsSTOP ");
  long timeDriving = D1miniTimeStop - D1miniTimeStart;
  if (BlindsDirection == 0) { // 0 = up
    timeHeightStatus = timeHeightStatus - timeDriving;
    if (timeHeightStatus < 0) {
      timeHeightStatus = 0;
    }
    timeSlatStatus = timeSlatStatus - timeDriving;
    if (timeSlatStatus < 0) {
      timeSlatStatus = 0;
    }
  }
  if (BlindsDirection == 1) { // 1 = down
    timeHeightStatus = timeHeightStatus + timeDriving;
    if (timeHeightStatus > timeHeight) {
      timeHeightStatus = timeHeight;
    }
    timeSlatStatus = timeSlatStatus + timeDriving;
    if (timeSlatStatus > timeSlat) {
      timeSlatStatus = timeSlat;
    }
  }
  float calculation1 = (100 / (float)timeHeight) * timeHeightStatus;
  HeightStatus = calculation1;
  float calculation2 = (100 / (float)timeSlat) * timeSlatStatus;
  SlatStatus = calculation2;
  BlindsStatusSending();
}

void BlindsStatusSending() {
  Serial.print("Write - 3/4/0 - HeightStatus: ");
  Serial.print(HeightStatus);
  Serial.print(" % = ");
  Serial.print(timeHeightStatus);
  Serial.println(" ms ");
  UDPipaction2 = 1;
  UDPdpt = 2; // 1=1bit 2=1Byte 3=2Byte
  UDPipaction2 <<= 4;
  packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
  UDPgaMain = 3;
  UDPgaMiddle = 4;
  UDPgaSub = 0;
  UDPgaMain <<= 3;
  packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  UDPat = 0; //0[HEX]
  UDPnpci = 0; //0[HEX]
  UDPat <<= 7;
  UDPnpci <<= 6;
  packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
  packetBufferWrite[16] = 128;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
  char calculation1byte = HeightStatus * constant1byte;
  packetBufferWrite[17] = calculation1byte;
  Udp.beginPacket(ipMulti, portMulti);
  Udp.write(packetBufferWrite, 18);
  Udp.endPacket();
  delay(5);
  Serial.print("Write - 3/5/0 - SlatStatus: ");
  Serial.print(SlatStatus);
  Serial.print(" % = ");
  Serial.print(timeSlatStatus);
  Serial.println(" ms ");
  UDPipaction2 = 1;
  UDPdpt = 2; // 1=1bit 2=1Byte 3=2Byte
  UDPipaction2 <<= 4;
  packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
  UDPgaMain = 3;
  UDPgaMiddle = 5;
  UDPgaSub = 0;
  UDPgaMain <<= 3;
  packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  UDPat = 0; //0[HEX]
  UDPnpci = 0; //0[HEX]
  UDPat <<= 7;
  UDPnpci <<= 6;
  packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
  packetBufferWrite[16] = 128;  // APCI(Type) [+2 bit], DATA/APCI [6 bit]
  calculation1byte = SlatStatus * constant1byte;
  packetBufferWrite[17] = calculation1byte;
  Udp.beginPacket(ipMulti, portMulti);
  Udp.write(packetBufferWrite, 18);
  Udp.endPacket();
}
