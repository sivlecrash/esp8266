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
// Relay1-on Relay2-off = blinds down
// Relay1-on Relay2-on = blinds up
// Relay1-off Relay2-off = blinds stop
long timeheight = 38000; // čas žaluzie dolů (ms) změnit dle žaluzie
long timeslat = 1300; // čas přejezdu lamely (ms) změnit dle žaluzie
long timeheightreset = timeheight + timeslat;
long timepart = timeslat / 5;
long timestatus = 0; // poslední nastavená (ms)
long heightstatus; // poslední nastavená (%)
long slatstatus; // poslední nastavená (%)
boolean drivingdirection = false; // true=down  false=up
long timeset; // vypočítaný čas motoru
int height_set;
int slat_set;
long time_down;
long time_up;
// long = 4Byte EIS11 DPT12.* 0...4294967295  unsignet value
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

    //--------------READ---------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 3) && (gamiddle == 4) && (gasub == 0)) {  // if "1bit" + "Read" + "3/4/0" - heightstatus
      Serial.print("Response - 3/4/0 - heightstatus - ");
      Serial.print(heightstatus);
      Serial.print(" % = ");
      Serial.print(timestatus);
      Serial.print(" ms ");
      Serial.print(" slat status: ");
      Serial.print(slatstatus);
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
      packetBufferWrite[17] = heightstatus * constant1byte;;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 18);
      Udp.endPacket();
    }

    //--------------READ---------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 3) && (gamiddle == 5) && (gasub == 0)) {  // if "1bit" + "Read" + "3/5/0" - slatstatus
      Serial.print("Response - 3/5/0 - slatstatus - ");
      Serial.print(heightstatus);
      Serial.print(" % = ");
      Serial.print(timestatus);
      Serial.print(" ms ");
      Serial.print(" slat status: ");
      Serial.print(slatstatus);
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
      packetBufferWrite[17] = slatstatus * constant1byte;;
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 18);
      Udp.endPacket();
    }

    //--------------WRITE---------------//
    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 0) && (gasub == 0)) {  // if "1bit" + "Write" + "3/0/0"
      if (data16 == 0) {
        Serial.println("Move - 0 - up");
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
        delay(timeheightreset);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = 0;
        heightstatus = 0;
        slatstatus = 0;
        BlindsStatusSending();
      }
      else if (data16 == 1) {
        Serial.println("Move - 1 - down");
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
        delay(timeheightreset);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = timeheight;
        heightstatus = 100;
        slatstatus = 100;
        BlindsStatusSending();
      }
      else {
        Serial.println("Move - ??");
      }
    }

    //--------------WRITE---------------//
    if  ((tlength == 1) && (apci == 2) && (gamain == 3) && (gamiddle == 1) && (gasub == 0)) {  // if "1bit" + "Write" + "3/1/0"
      if (data16 == 0) {
        Serial.println("Step - 0 - up");
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
        delay(timepart);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        long timeAll = timestatus - timepart;

        if (timepart >= timeheight) {
          timestatus = 0;
          slatstatus = 0;
          heightstatus = 0;
          Serial.print("SP11 height status: ");
          BlindsStatusSending();
        }
        else if ((timeAll < timeheight) && (timepart >= timeslat)) {
          float calculationheight = (100 / (float)timeheight) * timeAll;   //float calculationheight = (100 / (float)timeheight) * timestatus;
          heightstatus = calculationheight;
          timestatus = timeAll;
          slatstatus = 0;
          drivingdirection = true; // true=down  false=up
          Serial.print("SP21 height status: ");
          BlindsStatusSending();
        }
        else if (((timeAll < (timeheight - timestatus))) && (timepart >= timeslat)) {
          timestatus = timeAll;
          float calculation = (100 / (float)timeheight) * timestatus;
          heightstatus = calculation;
          slatstatus = 0;
          drivingdirection = true; // true=down  false=up
          Serial.print("SP12 height status: ");
          BlindsStatusSending();
        }
        else if (timepart < timeslat) {            //    else if (((timeAll < (timeheight - timestatus))) && (timepart < timeslat)) {
          timestatus = timeAll;
          float calculationheight = (100 / (float)timeheight) * timestatus;
          heightstatus = calculationheight;
          float calculationslat = slatstatus - ((100 / (float)timeslat) * timepart);
          slatstatus = calculationslat;
          drivingdirection = true; // true=down  false=up

          if (heightstatus <= 0) {
            heightstatus = 0;
            timestatus = 0;
          }
          else {

          }
          if  (slatstatus <= 0) {
            slatstatus = 0;
          }
          else {

          }
          Serial.print("SP13 height status: ");
          BlindsStatusSending();
        }
        else {
          Serial.println("Motor A&B STOP 4");
          digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        }
      }
      else if (data16 == 1) {
        Serial.println("Step - 1 - down");
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
        delay(timepart);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        long timeAll = timepart + timestatus;
        if (timeAll >= timeheight) {
          timestatus = timeheight;
          heightstatus = 100;
          slatstatus = 100;
          drivingdirection = true; // true=down  false=up
          Serial.print("SP8 height status: ");
          BlindsStatusSending();
        }
        else if (((timeAll < (timeheight - timestatus))) && (timepart >= timeslat)) {
          timestatus = timeAll;
          float calculation = (100 / (float)timeheight) * timestatus;
          heightstatus = calculation;
          slatstatus = 100;
          drivingdirection = true; // true=down  false=up
          Serial.print("SP9 height status: ");
          BlindsStatusSending();
        }
        else if (timepart < timeslat) {          //    else if (((timeAll < (timeheight - timestatus))) && (timepart < timeslat)) {
          timestatus = timeAll;
          float calculationheight = (100 / (float)timeheight) * timestatus;
          heightstatus = calculationheight;
          float calculationslat = slatstatus + ((100 / (float)timeslat) * timepart);
          slatstatus = calculationslat;
          drivingdirection = true; // true=down  false=up
          if (heightstatus >= 100) {
            heightstatus = 100;
            timestatus = timeheight;
          }
          else {

          }
          if  (slatstatus >= 100) {
            slatstatus = 100;
          }
          else {

          }
          Serial.print("SP10 height status: ");
          BlindsStatusSending();
        }
        else {
          Serial.println("Motor A&B STOP 3");
          digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        }
      }
      else {
        Serial.println("Step - ??");
      }
    }

    //--------------WRITE---------------//
    if  ((tlength == 2) && (apci == 2) && (gamain == 3) && (gamiddle == 2) && (gasub == 0)) {  // if "1Byte" + "Write" + "3/2/0"
      Serial.print("Write - 3/2/0 - Height_set ");
      height_set = data17 / constant1byte;
      Serial.print(height_set);
      timeset = (timeheight / 100) * height_set;
      Serial.print(" timeset:");
      Serial.println(timeset);

      if (timeset > timestatus) {
        long timedown = timeset - timestatus;
        Serial.print("start motor dolu, cas: ");
        Serial.println(timedown);
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
        delay(timedown);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = timeset;
        heightstatus = height_set;
        if (timedown > timeslat) {
          slatstatus = 100;
          Serial.print("X1");
        }
        else if ((timeslat > timedown) && (drivingdirection = false)) {
          float calculation = (100 / (float)timeslat) * timedown;
          slatstatus = calculation;
          Serial.print("X2");
        }
        else {
          slatstatus = 100;
          Serial.print("X3");
        }
        Serial.print("SP1 height status: ");
        BlindsStatusSending();
      }
      else if (timeset < timestatus) {
        long timeup = timestatus - timeset;
        Serial.print("start motor nahoru cas: ");
        Serial.println(timeup);
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
        delay(timeup);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = timeset;
        heightstatus = height_set;
        if (timeup > timeslat) {
          slatstatus = 0;
        }
        else if ((timeslat > timeup) && (drivingdirection = true)) {
          float calculation = (100 / (float)timeslat) * (timeslat - timeup);
          slatstatus = calculation;
        }
        else {
          slatstatus = 0;
        }
        Serial.print("SP2 height status: ");
        BlindsStatusSending();
      }
      else {
        Serial.println("Motor A&B STOP 1");
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        Serial.print("SP3 height status: ");
        BlindsStatusSending();
      }
    }

    //--------------WRITE---------------//
    if  ((tlength == 2) && (apci == 2) && (gamain == 3) && (gamiddle == 3) && (gasub == 0)) {  // if "1Byte" + "Write" + "3/3/0"
      Serial.print("Write - 3/2/0 - Slat_set ");
      slat_set = data17 / constant1byte;
      if (slat_set > slatstatus) {
        long timedownslat = (timeslat / 100) * (slat_set - slatstatus);
        Serial.print("start motor dolu, cas: ");
        Serial.println(timedownslat);
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
        delay(timedownslat);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = timestatus + timedownslat;
        slatstatus = slat_set;
        float calculation = (100 / (float)timeheight) * timestatus;
        heightstatus = calculation;
        Serial.print("SP4 height status: ");
        BlindsStatusSending();
      }
      else if ((slat_set < slatstatus) && (timestatus >= timeslat)) {
        long timeupslat = (timeslat / 100) * (slatstatus - slat_set);
        Serial.print("start motor nahoru cas: ");
        Serial.println(timeupslat);
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
        delay(timeupslat);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = timestatus - timeupslat;
        slatstatus = slat_set;
        float calculation = (100 / (float)timeheight) * timestatus;
        heightstatus = calculation;
        Serial.print("SP5 height status: ");
        BlindsStatusSending();
      }
      else if ((slat_set < slatstatus) && (timestatus < timeslat)) {
        long timeupslat2 = timestatus;
        Serial.print("start motor nahoru cas: ");
        Serial.println(timeupslat2);
        digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
        delay(timeupslat2);
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        timestatus = 0;
        slatstatus = 0;
        heightstatus = 0;
        Serial.print("SP6 height status: ");
        BlindsStatusSending();
      }
      else {
        Serial.println("Motor A&B STOP 2");
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
        Serial.print("SP7 height status: ");
        BlindsStatusSending();
      }
    }
    //--------------WRITE---------------//
    if  ((tlength == 2) && (apci == 2) && (gamain == 3) && (gamiddle == 7) && (gasub == 0)) {  // if "1Byte" + "Write" + "3/7/0"
      Serial.print("Write - 3/7/0 - time_down ");
      data4b17 = data17;
      data4b18 = data18;
      data4b19 = data19;
      data4b20 = data20;
      data4b17 <<= 24;
      data4b18 <<= 16;
      data4b19 <<= 8;
      time_down = data4b17 | data4b18 | data4b19 | data4b20; // 0...4294967295
      Serial.print("start motor dolu cas: ");
      Serial.println(time_down);
      digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, LOW);
      delay(time_down);
      digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
      long timeAll = time_down + timestatus;
      if (timeAll >= timeheight) {
        timestatus = timeheight;
        heightstatus = 100;
        slatstatus = 100;
        drivingdirection = true; // true=down  false=up
        Serial.print("SP8 height status: ");
        BlindsStatusSending();
      }
      else if (((timeAll < (timeheight - timestatus))) && (time_down >= timeslat)) {
        timestatus = timeAll;
        float calculation = (100 / (float)timeheight) * timestatus;
        heightstatus = calculation;
        slatstatus = 100;
        drivingdirection = true; // true=down  false=up
        Serial.print("SP9 height status: ");
        BlindsStatusSending();
      }
      else if (time_down < timeslat) {          //    else if (((timeAll < (timeheight - timestatus))) && (time_down < timeslat)) {
        timestatus = timeAll;
        float calculationheight = (100 / (float)timeheight) * timestatus;
        heightstatus = calculationheight;
        float calculationslat = slatstatus + ((100 / (float)timeslat) * time_down);
        slatstatus = calculationslat;
        drivingdirection = true; // true=down  false=up
        if (heightstatus >= 100) {
          heightstatus = 100;
          timestatus = timeheight;
        }
        else {

        }
        if  (slatstatus >= 100) {
          slatstatus = 100;
        }
        else {

        }
        Serial.print("SP10 height status: ");
        BlindsStatusSending();
      }
      else {
        Serial.println("Motor A&B STOP 3");
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
      }
    }

    //--------------WRITE---------------//
    if  ((tlength == 5) && (apci == 2) && (gamain == 3) && (gamiddle == 6) && (gasub == 0)) {  // if "4Byte" + "Write" + "3/6/0"
      Serial.print("Write - 3/6/0 - time_up ");
      data4b17 = data17;
      data4b18 = data18;
      data4b19 = data19;
      data4b20 = data20;
      data4b17 <<= 24;
      data4b18 <<= 16;
      data4b19 <<= 8;
      time_up = data4b17 | data4b18 | data4b19 | data4b20; // 0...4294967295
      Serial.print("start motor nahoru cas: ");
      Serial.println(time_up);
      digitalWrite(Relay1Pin, HIGH); digitalWrite(Relay2Pin, HIGH);
      delay(time_up);
      digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
      long timeAll = timestatus - time_up;

      if (time_up >= timeheight) {
        timestatus = 0;
        slatstatus = 0;
        heightstatus = 0;
        Serial.print("SP11 height status: ");
        BlindsStatusSending();
      }
      else if ((timeAll < timeheight) && (time_up >= timeslat)) {
        float calculationheight = (100 / (float)timeheight) * timeAll;   //float calculationheight = (100 / (float)timeheight) * timestatus;
        heightstatus = calculationheight;
        timestatus = timeAll;
        slatstatus = 0;
        drivingdirection = true; // true=down  false=up
        Serial.print("SP21 height status: ");
        BlindsStatusSending();
      }
      else if (((timeAll < (timeheight - timestatus))) && (time_up >= timeslat)) {
        timestatus = timeAll;
        float calculation = (100 / (float)timeheight) * timestatus;
        heightstatus = calculation;
        slatstatus = 0;
        drivingdirection = true; // true=down  false=up
        Serial.print("SP12 height status: ");
        BlindsStatusSending();
      }
      else if (time_up < timeslat) {            //    else if (((timeAll < (timeheight - timestatus))) && (time_up < timeslat)) {
        timestatus = timeAll;
        float calculationheight = (100 / (float)timeheight) * timestatus;
        heightstatus = calculationheight;
        float calculationslat = slatstatus - ((100 / (float)timeslat) * time_up);
        slatstatus = calculationslat;
        drivingdirection = true; // true=down  false=up

        if (heightstatus <= 0) {
          heightstatus = 0;
          timestatus = 0;
        }
        else {

        }
        if  (slatstatus <= 0) {
          slatstatus = 0;
        }
        else {

        }
        Serial.print("SP13 height status: ");
        BlindsStatusSending();
      }
      else {
        Serial.println("Motor A&B STOP 4");
        digitalWrite(Relay1Pin, LOW); digitalWrite(Relay2Pin, LOW);
      }
    }
    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }
}
void BlindsStatusSending() {
  Serial.print(heightstatus);
  Serial.print(" % = ");
  Serial.print(timestatus);
  Serial.print(" ms ");
  Serial.print(" slat status: ");
  Serial.print(slatstatus);
  Serial.println(" %");
  Serial.print("Write - 3/4/0 - heightstatus ");
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
  char calculation1byte = heightstatus * constant1byte;
  packetBufferWrite[17] = calculation1byte;
  Serial.println(packetBufferWrite[17]);
  Udp.beginPacket(ipMulti, portMulti);
  Udp.write(packetBufferWrite, 18);
  Udp.endPacket();
  
  delay(5);
  Serial.print("Write - 3/5/0 - slatstatus ");
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
  calculation1byte = slatstatus * constant1byte;
  packetBufferWrite[17] = calculation1byte;
  Serial.println(packetBufferWrite[17]);
  Udp.beginPacket(ipMulti, portMulti);
  Udp.write(packetBufferWrite, 18);
  Udp.endPacket();
}
