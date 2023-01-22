//  *********************************
//  **        Release Notes        **
//  *********************************
//
//  V0.2 - Dec/29/2022
//     - Added RDS Function
//     - Performance improvements
//
//  V0.1 - Dec/27/2022
//     - Initial Release without RDS and save configs
//
//  *********************************
//  **          Libraries          **
//  *********************************
//
//  Library TFT_eSPI you MUST download from here :
//        For Lilygo T-Display-S3: https://github.com/Xinyuan-LilyGO/T-Display-S3 (See "lib" directory)
//        For Lilygo T-Embed: https://github.com/Xinyuan-LilyGO/T-Embed (See "lib" directory)
//  Library FastLED you may download from here :
//        For Lilygo T-Display-S3: https://github.com/Xinyuan-LilyGO/T-Display-S3 (See "lib" directory)
//        For Lilygo T-Embed: https://github.com/Xinyuan-LilyGO/T-Embed (See "lib" directory)
//  Library RotaryEncoder you may download from here : https://github.com/mathertel/RotaryEncoder
//  Library SI4735 you may download from here   : https://github.com/pu2clr/SI4735
//
//  *********************************
//  **       Connections etc.      **
//  *********************************
//
//  Lilygo T-Display S3
//
//  |--------------|------------|------------|------------|
//  |   Lilygo     |   Si4735   |  Encoder   |   Audio    |
//  | T-Display S3 |            |            | Amplifier  |        
//  |--------------|------------|------------|------------|        
//  |     3V3      |    Vcc     |            |    Vcc     |        
//  |     GND      |    GND     |     2,4    |    GND     |        Encoder        1,2,3        
//  |     21       |            |     5      |            |        Encoder switch 4,5
//  |     16       |   Reset    |            |            |
//  |     43       |    SDA     |            |            |
//  |     44       |    SCL     |            |            |
//  |      1       |            |      1     |            |
//  |      2       |            |      3     |            |
//  |              |    LOut    |            |    LIn     |
//  |              |    ROut    |            |    RIn     |
//  |     17 Mute  |            |            |    Mute    |
//  |--------------|------------|------------|------------|
//
//
//  Lilygo T-Embed
//
//  |--------------|------------|------------|
//  |    Lilygo    |   Si4735   |   Audio    |
//  |   T-Embeded  |            | Amplifier  |
//  |--------------|------------|------------|
//  |     3V3      |    Vcc     |    Vcc     |
//  |     GND      |    GND     |    GND     |
//  |              |            |            |
//  |     16       |   Reset    |            |
//  |     18       |    SDA     |            |
//  |      8       |    SCL     |            |
//  |              |            |            |
//  |              |            |            |
//  |              |    LOut    |    LIn     |
//  |              |    ROut    |    RIn     |
//  |     17 Mute  |            |    Mute    |
//  |--------------|------------|------------|
//

#define I_use_T_Display_S3     // Comment this line if you use a Lilygo T-Embed 
// #define SI4735_EMUL         // Uncomment if you don't have a si4735/si4732 and just want to look and feel this sketch.

#include <Wire.h>
#include <SI4735.h>         // https://github.com/pu2clr/SI4735
#ifndef I_use_T_Display_S3
#include <FastLED.h>
#endif
#include "TFT_eSPI.h"       // See instructions on Lilygo github (https://github.com/Xinyuan-LilyGO/T-Display-S3 or https://github.com/Xinyuan-LilyGO/T-Embed)
#include <RotaryEncoder.h>  // https://github.com/mathertel/RotaryEncoder
#include <OneButton.h>      // https://github.com/mathertel/OneButton


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

#define PIN_IN1          2
#define PIN_IN2          1

#ifdef I_use_T_Display_S3
#define BUTTON01        21
#else
#define BUTTON01         0
#endif

#ifdef I_use_T_Display_S3
#define I2C_SDA         43
#define I2C_SCL         44
#else
#define I2C_SDA         18
#define I2C_SCL         8
#endif

#define RESET_PIN       16
#define AUDIO_MUTE      17

#define STRENGTH_CHECK_TIME 1500
#define RDS_CHECK_TIME 90
#define currentVOL      45
#define FM_FUNCTION      0
uint8_t currentMode = FM_FUNCTION;
uint16_t minimumFMFreq =  7600; // Minimum frequency of the band
uint16_t maximumFMFreq = 10800; // maximum frequency of the band
uint16_t currentFMFreq = 10250; // Default frequency or current frequency
uint8_t  currentFMStep =    10; // Default step (increment and decrement)

#ifndef I_use_T_Display_S3
#define NUM_LEDS         7
#define DATA_PIN        42
#define CLOCK_PIN       45
CRGB leds[NUM_LEDS];
#endif

#define color1      0xC638
#define color2      0xC638

int value=currentFMFreq;
int minimal=minimumFMFreq;
int maximal=maximumFMFreq;
int strength=0;
bool stereoPilot=0;
bool newStereoPilot=0;
uint8_t rssi=0;
uint8_t snr=0;
int newStrength=10;
long lastStrengthCheck = millis();
long lastRDSCheck = millis();
uint16_t sta[6]={9850,10030,9250,9170,10370,10250};
const int lastStationMenu = (sizeof sta/sizeof (sta[0])) - 1;
int selectedStation = 0;

char *rdsMsg;
char *stationName;
char *rdsTime;
char bufferStationName[50];
char bufferRdsMsg[100];
char bufferRdsTime[32];

float freq=currentFMFreq;

RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

OneButton button(BUTTON01, true);

SI4735 si4735;


bool muted       =  0;
bool volAdj      =  0;
bool stationsMenu = 0;
bool setStationsMenu = 0;
int  volLevel    = 45;
uint8_t currentVolStep = 1;

// save the millis when a press has started.
unsigned long pressStartTime;

void IRAM_ATTR checkTicks() {
  // include all buttons here to be checked
  button.tick(); // just call tick() to check the state.
  encoder.tick();
}

// this function will be called when the button was pressed 1 time only.
void singleClick() {
  if (setStationsMenu) {
    #ifndef SI4735_EMUL
    freq=si4735.getFrequency();
    #endif
    sta[selectedStation]=int(freq);
    setStationsMenu=!setStationsMenu;
  } else if (stationsMenu) {
    #ifdef SI4735_EMUL
    freq=sta[selectedStation];
    #else
    si4735.setFrequency(sta[selectedStation]);
    freq=si4735.getFrequency();
    cleanBfoRdsInfo();
    #endif
    stationsMenu=!stationsMenu;
  } 
  else volAdj=!volAdj;
  drawSprite();
} // singleClick


// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick() {
  stationsMenu = !stationsMenu;
  if (volAdj) volAdj=!volAdj;
  drawSprite();
} // doubleClick


// this function will be called when the button was pressed multiple times in a short timeframe.
void multiClick() {
  int n = button.getNumberClicks();
  if (n == 3) {
    muted=!muted;
    #ifndef SI4735_EMUL
    if (muted) {
      si4735.setAudioMute(muted);
    } else {
      si4735.setAudioMute(muted);
      si4735.setVolume(volLevel);
    }
    #endif
    drawSprite();
  }
} // multiClick


// this function will be called when the button was held down for 1 second or more.
void pressStart() {
  pressStartTime = millis() - 1000; // as set in setPressTicks()
  if (volAdj) volAdj=!volAdj;
  setStationsMenu=!setStationsMenu;
  drawSprite();
} // pressStart()


// this function will be called when the button was released after a long hold.
void pressStop() {
  
} // pressStop()

void setup() {

  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);

  tft.begin();
  tft.writecommand(0x11);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  // setup interrupt routine
  // when not registering to the interrupt the sketch also works when the tick is called frequently.
  attachInterrupt(digitalPinToInterrupt(BUTTON01), checkTicks, CHANGE);

  // link the xxxclick functions to be called on xxxclick event.
  button.attachClick(singleClick);
  button.attachDoubleClick(doubleClick);
  button.attachMultiClick(multiClick);

  button.setPressTicks(1000); // that is the time when LongPressStart is called
  button.attachLongPressStart(pressStart);
  button.attachLongPressStop(pressStop);
 
  #ifndef I_use_T_Display_S3
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  #endif

  #ifdef SI4735_EMUL
  int16_t si4735Addr = 55;
  #else
  Wire.begin(I2C_SDA,I2C_SCL);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);
  #endif

  if ( si4735Addr == 0 ) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("Si4735 not detected");
    while (1);
  }
  
  #ifndef SI4735_EMUL
  if (si4735Addr == 17)
  {
    si4735.setDeviceI2CAddress(0);
  }
  else
  {
    si4735.setDeviceI2CAddress(1);
  }
  delay(100);
  si4735.setup(RESET_PIN, FM_FUNCTION);
  si4735.setFM(minimumFMFreq, maximumFMFreq, currentFMFreq, currentFMStep);
  delay(500);
  si4735.setFMDeEmphasis(1);
  si4735.setVolume(currentVOL);
  si4735.RdsInit();
  si4735.setRdsConfig(1, 2, 2, 2, 2);
  #endif

  spr.createSprite(320,170);
  spr.setTextDatum(4);
  spr.setSwapBytes(true);
  spr.setFreeFont(&Orbitron_Light_24);
  spr.setTextColor(color1,TFT_BLACK);

  #ifndef I_use_T_Display_S3
  leds[0] = CRGB::Red;
  leds[1] = CRGB::White;
  leds[2] = CRGB::Red;
  leds[3] = CRGB::Green;
  leds[4] = CRGB::Red;
  leds[5] = CRGB::Blue;
  leds[6] = CRGB::Red;
  FastLED.show();
  #endif
  drawSprite();
}

void readEncoder() {

  static int pos = 0;

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    if(newPos>pos) {
      if (volAdj) {
        volLevel=volLevel+currentVolStep;
        if (volLevel > 63) volLevel=63;
        #ifndef SI4735_EMUL
        if (!muted) si4735.setVolume(volLevel);
        #endif
      } else if (stationsMenu or setStationsMenu) {
        selectedStation=selectedStation-1;
        if (selectedStation < 0) selectedStation=0;
      } else {
        #ifndef SI4735_EMUL
        if (si4735.getCurrentFrequency() < maximumFMFreq) {
          si4735.frequencyUp();
          cleanBfoRdsInfo();
        }
        #else
        freq=freq+currentFMStep;
        if (freq>maximumFMFreq) freq=maximumFMFreq;
        #endif
      }
    }
    if(newPos<pos) {
      if (volAdj) {
        volLevel=volLevel-currentVolStep;
        if (volLevel < 0) volLevel=0;
        #ifndef SI4735_EMUL        
        if (!muted) si4735.setVolume(volLevel);
        #endif
      }
      else if (stationsMenu or setStationsMenu) {
        selectedStation=selectedStation+1;
        if (selectedStation > lastStationMenu) selectedStation=lastStationMenu;
      } else {
        #ifndef SI4735_EMUL
        if (si4735.getCurrentFrequency() > minimumFMFreq) {
          si4735.frequencyDown();
          cleanBfoRdsInfo();
        } 
        #else
        freq=freq-currentFMStep;
        if (freq<minimumFMFreq) freq=minimumFMFreq;
        #endif
      }
    }
    
    pos = newPos;

    drawSprite();
  } 

  if ((millis() - lastStrengthCheck) > STRENGTH_CHECK_TIME) {
    #ifndef SI4735_EMUL
    si4735.getCurrentReceivedSignalQuality();
    rssi=si4735.getCurrentRSSI();
    snr= si4735.getCurrentSNR();
    #else
    rssi=random(11,64);
    #endif
    if (currentMode != FM_FUNCTION) {
      //dBuV to S point conversion HF
      if ((rssi >= 0) and (rssi <=  1)) newStrength =  1;  // S0
      if ((rssi >  1) and (rssi <=  1)) newStrength =  2;  // S1
      if ((rssi >  2) and (rssi <=  3)) newStrength =  3;  // S2
      if ((rssi >  3) and (rssi <=  4)) newStrength =  4;  // S3
      if ((rssi >  4) and (rssi <= 10)) newStrength =  5;  // S4
      if ((rssi > 10) and (rssi <= 16)) newStrength =  6;  // S5
      if ((rssi > 16) and (rssi <= 22)) newStrength =  7;  // S6
      if ((rssi > 22) and (rssi <= 28)) newStrength =  8;  // S7
      if ((rssi > 28) and (rssi <= 34)) newStrength =  9;  // S8
      if ((rssi > 34) and (rssi <= 44)) newStrength = 10;  // S9
      if ((rssi > 44) and (rssi <= 54)) newStrength = 11;  // S9 +10
      if ((rssi > 54) and (rssi <= 64)) newStrength = 12;  // S9 +20
      if ((rssi > 64) and (rssi <= 74)) newStrength = 13;  // S9 +30
      if ((rssi > 74) and (rssi <= 84)) newStrength = 14;  // S9 +40
      if ((rssi > 84) and (rssi <= 94)) newStrength = 15;  // S9 +50
      if  (rssi > 94)                   newStrength = 16;  // S9 +60
      if  (rssi > 95)                   newStrength = 17;  //>S9 +60
    }
    else
    {
      //dBuV to S point conversion FM
      if  (rssi <  1)                   newStrength =  1;
      if ((rssi >  1) and (rssi <=  2)) newStrength =  7;  // S6
      if ((rssi >  2) and (rssi <=  8)) newStrength =  8;  // S7
      if ((rssi >  8) and (rssi <= 14)) newStrength =  9;  // S8
      if ((rssi > 14) and (rssi <= 24)) newStrength = 10;  // S9
      if ((rssi > 24) and (rssi <= 34)) newStrength = 11;  // S9 +10
      if ((rssi > 34) and (rssi <= 44)) newStrength = 12;  // S9 +20
      if ((rssi > 44) and (rssi <= 54)) newStrength = 13;  // S9 +30
      if ((rssi > 54) and (rssi <= 64)) newStrength = 14;  // S9 +40
      if ((rssi > 64) and (rssi <= 74)) newStrength = 15;  // S9 +50
      if  (rssi > 74)                   newStrength = 16;  // S9 +60
      if  (rssi > 76)                   newStrength = 17;  //>S9 +60
      #ifndef SI4735_EMUL
      newStereoPilot=si4735.getCurrentPilot();
      #endif
    }

    if (strength != newStrength or stereoPilot != newStereoPilot) {
      strength = newStrength;
      stereoPilot = newStereoPilot;
      drawSprite();
    }
    lastStrengthCheck = millis();
  }

  if ((millis() - lastRDSCheck) > RDS_CHECK_TIME) {
    if ((currentMode == FM_FUNCTION) and (snr >= 12)) checkRDS();
    lastRDSCheck = millis();
  }  
  
}

void drawSprite()
{
  #ifndef SI4735_EMUL
  freq=si4735.getFrequency();
  #endif
  spr.fillSprite(TFT_BLACK);
  spr.setTextColor(TFT_WHITE,TFT_BLACK);
  
  if (!volAdj) {
    spr.drawFloat(freq/100.00,1,160,60,7);
  } else {
    spr.drawString("VOL:",100,74,2);
    spr.drawString(String(volLevel),160,64,7);
  }
  spr.setFreeFont(&Orbitron_Light_24);
  spr.drawString("FM Radio",160,12);
  spr.drawString("STATIONS",38,14,2);
  // spr.drawRoundRect(1,1,76,110,4,0xAD55);

  if (stationsMenu) spr.drawRoundRect(1,1,76,110,4,TFT_RED);
  else if (setStationsMenu) spr.drawRoundRect(1,1,76,110,4,TFT_GREEN);
  else spr.drawRoundRect(1,1,76,110,4,0xAD55);

  spr.drawRoundRect(240,20,76,22,4,TFT_WHITE);
  
  spr.drawRect(290,6,20,9,TFT_WHITE);
  spr.fillRect(291,7,12,7,0x34CD);
  spr.fillRect(310,8,2,5,TFT_WHITE);
  
  spr.setTextFont(0);
  spr.setTextColor(0xBEDF,TFT_BLACK);
  for(int i=0;i<6;i++){
    float station=sta[i];
    spr.drawFloat(station/100,2,38,32+(i*12));
    spr.fillCircle(16,31+(i*12),2,0xFBAE);
    if (stationsMenu or setStationsMenu)
      spr.fillTriangle(5,28+selectedStation*12,11,31+selectedStation*12,5,34+selectedStation*12,TFT_WHITE);
  }
  spr.setTextColor(TFT_WHITE,TFT_BLACK);
  
  spr.drawString("SIGNAL:",266,54);
  spr.drawString("MUTED",260,102,2);
  spr.fillRoundRect(288,96,20,20,3,0xCC40);
  
  if(muted==1)
    spr.fillCircle(297,105,6,TFT_WHITE);
    
  
  for(int i=0;i<strength;i++)
    if (i<10)
      spr.fillRect(244+(i*4),80-(i*1),2,4+(i*1),0x3526);
    else
      spr.fillRect(244+(i*4),80-(i*1),2,4+(i*1),TFT_RED);
  
  
  spr.fillTriangle(156,112,160,122,164,112,TFT_RED);
  
    
  int temp=(freq/10.00)-20;
  for(int i=0;i<40;i++)
  {
    if (!(temp<minimumFMFreq/10.00 or temp>maximumFMFreq/10.00)) {
      if((temp%10)==0){
        spr.drawLine(i*8,170,i*8,140,color1);
        
        spr.drawLine((i*8)+1,170,(i*8)+1,140,color1);
        spr.drawFloat(temp/10.0,1,i*8,130,2);
      } else if((temp%5)==0 && (temp%10)!=0) {
        spr.drawLine(i*8,170,i*8,150,color1);
        spr.drawLine((i*8)+1,170,(i*8)+1,150,color1);
        // spr.drawFloat(temp/10.0,1,i*8,144);
      } else {
        spr.drawLine(i*8,170,i*8,160,color1);
      }
    }
  
   temp=temp+1;
  }
  #ifndef SI4735_EMUL
  if (stereoPilot) {
    spr.setTextColor(TFT_RED,TFT_BLACK);
    spr.drawString("Stereo",275,31,2);
    spr.setTextColor(TFT_WHITE,TFT_BLACK);
  } else spr.drawString("Mono",275,31,2);
  #else
  spr.setTextColor(TFT_RED,TFT_BLACK);
  spr.drawString("Stereo",275,31,2);
  spr.setTextColor(TFT_WHITE,TFT_BLACK);
  #endif
  
  spr.drawLine(160,114,160,170,TFT_RED);

  // spr.setTextColor(TFT_MAGENTA,TFT_BLACK);
  spr.drawString(bufferStationName,160,102,4);
  // spr.setTextColor(TFT_WHITE,TFT_BLACK);

  spr.pushSprite(0,0);
 
}

void cleanBfoRdsInfo()
{
  bufferStationName[0]='\0';
}

void showRDSMsg()
{
  rdsMsg[35] = bufferRdsMsg[35] = '\0';
  if (strcmp(bufferRdsMsg, rdsMsg) == 0)
    return;
}

void showRDSStation()
{
  if (strcmp(bufferStationName, stationName) == 0 ) return;
  cleanBfoRdsInfo();
  strcpy(bufferStationName, stationName);
  drawSprite();
}

void showRDSTime()
{

  if (strcmp(bufferRdsTime, rdsTime) == 0)
    return;
}

void checkRDS()
{
  si4735.getRdsStatus();
  if (si4735.getRdsReceived())
  {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound())
    {
      rdsMsg = si4735.getRdsText2A();
      stationName = si4735.getRdsText0A();
      rdsTime = si4735.getRdsTime();
      // if ( rdsMsg != NULL )   showRDSMsg();
      if (stationName != NULL)         
          showRDSStation();
      // if ( rdsTime != NULL ) showRDSTime();
    }
  }
}

void loop() { 
  
  button.tick();
  encoder.tick();
  readEncoder();

}
