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


//WS2812B
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN D4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(203, PIN, NEO_GRB + NEO_KHZ800);

//Individual Address - 1.1.1 Area [4 bit] . Line [4 bit] . Bus device [1 byte]
uint8_t UDParea = 10;
uint8_t UDPline = 0;
uint8_t UDPdevice = 253;

//UDP
uint8_t UDPipaction2 = 0;
uint8_t UDPdpt = 0;
uint8_t UDPgaMain = 0;
uint8_t UDPgaMiddle = 0;
uint8_t UDPgaSub = 0;
uint8_t UDPat = 0;
uint8_t UDPnpci = 0;
uint8_t dpt251600set = 0; //[8 bit] Program settings
uint8_t dpt251600empty = 0; //[4 bit]
uint8_t dpt251600Vred = 0; //[1 bit] Shall specify whether the colour information red in the field R is valid or not.
uint8_t dpt251600Vgreen = 0; //[1 bit] Shall specify whether the colour information green in the field G is valid or not.
uint8_t dpt251600Vblue = 0; //[1 bit] Shall specify whether the colour information blue in the field B is valid or not.
uint8_t dpt251600Vintensity = 0; //[1 bit] Shall specify whether the colour information intensity in the field W is valid or not.
uint8_t dpt251500red = 0; //[8 bit] Colour Level Red
uint8_t dpt251500green = 0; //[8 bit] Colour Level Green
uint8_t dpt251500blue = 0; //[8 bit] Colour Level Blue
uint8_t dpt251500intensity = 0;  //[8 bit] Colour Level intensity
uint8_t Sdpt251600set = 0; //[8 bit] Status - Program settings
uint8_t Sdpt251600empty = 0; //[4 bit] Status -
uint8_t Sdpt251600Vred = 0; //[1 bit] Status - Shall specify whether the colour information red in the field R is valid or not.
uint8_t Sdpt251600Vgreen = 0; //[1 bit] Status - Shall specify whether the colour information green in the field G is valid or not.
uint8_t Sdpt251600Vblue = 0; //[1 bit] Status - Shall specify whether the colour information blue in the field B is valid or not.
uint8_t Sdpt251600Vintensity = 0; //[1 bit] Status - Shall specify whether the colour information intensity in the field W is valid or not.
uint8_t Sdpt251500red = 0; //[8 bit] Status - Colour Level Red
uint8_t Sdpt251500green = 0; //[8 bit] Status - Colour Level Green
uint8_t Sdpt251500blue = 0; //[8 bit] Status - Colour Level Blue
uint8_t Sdpt251500intensity = 0;  //[8 bit] Status - Colour Level intensity

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void setup() {
  Serial.begin(115200);

  //WS2812B
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif  // End of trinket special code
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  theaterChase(strip.Color(50, 50, 50), 50); // set white after power on

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
    if  ((tlength == 7) && (apci == 0) && (gamain == 6) && (gamiddle == 0) && (gasub == 0)) {  // if "6Byte" + "Read" + "6/0/0"
      Serial.print("Response - 6/0/0 - RGB ");
      UDPipaction2 = 1;
      UDPdpt = 3; // 1=1bit 2=1Byte 3=2Byte
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      UDPgaMain = 6;
      UDPgaMiddle = 0;
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
      packetBufferWrite[17] = Sdpt251600set; //[8 bit] Status - Program settings
      uint8_t a = Sdpt251600empty; //[4 bit] Status -
      uint8_t b = Sdpt251600Vred; //[1 bit] Status - Shall specify whether the colour information red in the field R is valid or not.
      uint8_t c = Sdpt251600Vgreen; //[1 bit] Status - Shall specify whether the colour information green in the field G is valid or not.
      uint8_t d = Sdpt251600Vblue; //[1 bit] Status - Shall specify whether the colour information blue in the field B is valid or not.
      uint8_t e = Sdpt251600Vintensity; //[1 bit] Status - Shall specify whether the colour information intensity in the field W is valid or not.
      a <<= 4;
      b <<= 3;
      c <<= 2;
      d <<= 1;
      packetBufferWrite[18] = a | b | c | d | e;
      packetBufferWrite[19] = Sdpt251500red; //[8 bit] Status - Colour Level Red
      packetBufferWrite[20] = Sdpt251500green; //[8 bit] Status - Colour Level Green
      packetBufferWrite[21] = Sdpt251500blue; //[8 bit] Status - Colour Level Blue
      packetBufferWrite[22] = Sdpt251500intensity;  //[8 bit] Status - Colour Level intensity

      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 19);
      Udp.endPacket();
    }

    //--------------WRITE---------------//
    if  ((tlength == 7) && (apci == 2) && (gamain == 6) && (gamiddle == 0) && (gasub == 0)) {  // if "6Byte" + "Write" + "6/0/0"
      Serial.print("Write - 6/0/0 - RGB ");
      dpt251600set = data17; //[8 bit] Program settings
      dpt251600empty = data18; //[4 bit]
      dpt251600Vred = data18; //[1 bit] Shall specify whether the colour information red in the field R is valid or not.
      dpt251600Vgreen = data18; //[1 bit] Shall specify whether the colour information green in the field G is valid or not.
      dpt251600Vblue = data18; //[1 bit] Shall specify whether the colour information blue in the field B is valid or not.
      dpt251600Vintensity = data18; //[1 bit] Shall specify whether the colour information intensity in the field W is valid or not.
      dpt251500red = data19; //[8 bit] Colour Level Red
      dpt251500green = data20; //[8 bit] Colour Level Green
      dpt251500blue = data21; //[8 bit] Colour Level Blue
      dpt251500intensity = data22;  //[8 bit] Colour Level intensity
      dpt251600empty >>= 4;
      dpt251600Vred <<= 4;
      dpt251600Vred >>= 7;
      dpt251600Vgreen <<= 5;
      dpt251600Vgreen >>= 7;
      dpt251600Vblue <<= 6;
      dpt251600Vblue >>= 7;
      dpt251600Vintensity <<= 7;
      dpt251600Vintensity >>= 7;
      Sdpt251600set = dpt251600set; //[8 bit] Status - Program settings
      Sdpt251600empty = dpt251600empty; //[4 bit] Status -
      Sdpt251600Vred = dpt251600Vred; //[1 bit] Status - Shall specify whether the colour information red in the field R is valid or not.
      Sdpt251600Vgreen = dpt251600Vgreen; //[1 bit] Status - Shall specify whether the colour information green in the field G is valid or not.
      Sdpt251600Vblue = dpt251600Vblue; //[1 bit] Status - Shall specify whether the colour information blue in the field B is valid or not.
      Sdpt251600Vintensity = dpt251600Vintensity; //[1 bit] Status - Shall specify whether the colour information intensity in the field W is valid or not.
      Sdpt251500red = dpt251500red; //[8 bit] Status - Colour Level Red
      Sdpt251500green = dpt251500green; //[8 bit] Status - Colour Level Green
      Sdpt251500blue = dpt251500blue; //[8 bit] Status - Colour Level Blue
      Sdpt251500intensity = dpt251500intensity;  //[8 bit] Status - Colour Level intensity

      if (dpt251600set == 0) {
        colorWipe(strip.Color(dpt251500red, dpt251500green, dpt251500blue), dpt251500intensity);
      }
      else if (dpt251600set == 1) {
        theaterChase(strip.Color(dpt251500red, dpt251500green, dpt251500blue), dpt251500intensity);
      }
      else if (dpt251600set == 2) {
        rainbow(20);
      }
      else if (dpt251600set == 3) {
        rainbowCycle(20);
      }
      else if (dpt251600set == 4) {
        theaterChaseRainbow(50);
      }
      else {
        colorWipe(strip.Color(70, 70, 70), 50);
      }
    }

    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
