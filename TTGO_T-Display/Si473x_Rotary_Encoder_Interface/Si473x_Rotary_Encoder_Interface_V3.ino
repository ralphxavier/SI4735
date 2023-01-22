// The sketches and documentation available here are based on Gert's sketch that made his work available for every one.
// I would like to thank to [Gert Baak](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino).
//
//  This sketch was made by Ralph Xavier
//  V3 Jun-06-2021 Initial Release.
//  This sketch is based on the SI4735 Library of Ricardo PU2CLR. Thanks for the very nice work.
//  This sketch uses a Rotary Encoder and TTGO T-Display (http://www.lilygo.cn/prod_view.aspx?TypeId=50044&Id=1126).
//  The radio is fully controlled by the Rotary Encoder
//  This sketch uses the Rotary Encoder Class implementation from Ben Buxton (the source code is included
//  together with this sketch).
//  ABOUT SSB PATCH:
//  This sketch will download a SSB patch to your SI4735 device (patch_init.h). It will take about 8KB of the Arduino memory.
//  In this context, a patch is a piece of software used to change the behavior of the SI4735 device.
//  There is little information available about patching the SI4735. The following information is the understanding of the author of
//  this project and it is not necessarily correct. A patch is executed internally (run by internal MCU) of the device.
//  Usually, patches are used to fixes bugs or add improvements and new features of the firmware installed in the internal ROM of the device.
//  Patches to the SI4735 are distributed in binary form and have to be transferred to the internal RAM of the device by
//  the host MCU (in this case Arduino). Since the RAM is volatile memory, the patch stored into the device gets lost when you turn off the system.
//  Consequently, the content of the patch has to be transferred again to the device each time after turn on the system or reset the device.
//
//  ATTENTION: The author of this project does not guarantee that procedures shown here will work in your development environment.
//  Given this, it is at your own risk to continue with the procedures suggested here.
//  This library works with the I2C communication protocol and it is designed to apply a SSB extension PATCH to CI SI4735-D60.
//  Once again, the author disclaims any liability for any damage this procedure may cause to your SI4735 or other devices that you are using.
//
//  Library TFT_eSPI you may download from here : https://github.com/Bodmer/TFT_eSPI
//  Library Rotary is provided with the program
//  Library SI4735 you may download from here   : https://github.com/pu2clr/SI4735
//
//  *********************************
//  **       Connections etc.      **
//  *********************************
//  |-------------|------------|------------|------------|
//  |    TTGO     |   Si4735   |  Encoder   |   Audio    |
//  |  T-Display  |            |            | Amplifier  |        
//  |-------------|------------|------------|------------|        
//  |     3V3     |    Vcc     |            |    Vcc     |        
//  |     GND     |    GND     |     2,4    |    GND     |        Encoder        1,2,3        
//  |      2      |            |     5      |            |        Encoder switch 4,5
//  |     25      |   Reset    |            |            |
//  |     21      |    SDA     |            |            |
//  |     22      |    SCL     |            |            |
//  |     33      |            |      1     |            |
//  |     32      |            |      3     |            |
//  |             |    LOut    |            |    LIn     |
//  |             |    ROut    |            |    RIn     |
//  |     27 Mute |            |            |    Mute    |
//  |-------------|------------|------------|------------|

#include <Arduino.h>
#include <SI4735.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "EEPROM.h"
#include "Rotary.h"
#include "DSEG7_Classic_Mini_Bold_20.h"
#include "DSEG7_Classic_Mini_Bold_30.h"
#include "Serif_bold_10.h"
#include "Serif_bold_15.h"
#include "Serif_bold_20.h"
#include "Serif_plain_10.h"
#include "Serif_plain_15.h"
#include "wave.h"

// =================================================
#define IhaveTDisplayTFT
// =================================================

// Test it with patch_init.h or patch_full.h. Do not try load both.
#include "patch_init.h" // SSB patch for whole SSBRX initialization string
//#include "patch_full.h"    // SSB patch for whole SSBRX full download

const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h

#define ESP32_I2C_SDA        21  // I2C bus pin on ESP32
#define ESP32_I2C_SCL        22  // I2C bus pin on ESP32
#define RESET_PIN            25
#define ENCODER_PIN_A        32  // http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
#define ENCODER_PIN_B        33
#define ENCODER_PUSH_BUTTON   2 
#define displayon             1 
#define displayoff            0 
#define AUDIO_MUTE           27

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define MIN_ELAPSED_TIME             100
#define MIN_ELAPSED_RSSI_TIME        150
#define MIN_ELAPSED_DISPL_TIME       5*60*1000  // 5 minutes
#define MIN_ELAPSED_ANIMATION_TIME   3*60*1000  // 3 minutes
// #define MIN_ELAPSED_RDS_TIME         5
#define DEFAULT_VOLUME               45  // change it for your favorite start sound volume
#define MIN_ELAPSED_VOLbut_TIME      1000

#define FM          0
#define LSB         1
#define USB         2
#define AM          3
#define SYNC        4

bool bfoOn          = false;
bool ssbLoaded      = false;

bool FREQbut       = false;
bool Decipoint     = false;
bool DISplay       = false;
bool DISplayAni    = false;
bool Mutestat      = false;
bool AGCgainbut    = false;
bool writingEeprom = false;

bool audioMuteOn = true;
bool audioMuteOff = false;
bool RDS = true; // RDS on  or  off
bool SEEK =  false;

int currentBFO;
int previousBFO = 0;
int nrbox       = 0;
int OldRSSI;
int NewRSSI;
int NewSNR;
int encBut;
int AGCgain     = 0;

long elapsedRSSI        = millis();
// long elapsedRDS         = millis();
// long stationNameElapsed = millis();
long DisplayOnTime      = millis();
long DisplayAniTime     = millis();
long VOLbutOnTime       = millis();

volatile int encoderCount  = 0;

uint16_t previousFrequency;
uint8_t currentStep        =  1;

uint8_t currentBFOStep     = 25;

uint8_t currentPRES        =  0;
uint8_t previousPRES       =  0;
uint8_t currentPRESStep    =  1;

int currentAGCgain         =  1;
int previousAGCgain        =  1;
uint8_t currentAGCgainStep =  1;
uint8_t MaxAGCgain;
uint8_t MaxAGCgainFM       = 26;
uint8_t MaxAGCgainAM       = 37;
uint8_t MinAGCgain         =  1;



int currentVOL             =  0;
int previousVOL            =  0;
uint8_t currentVOLStep     =  1;
uint8_t MaxVOL             = 63;
uint8_t MinVOL             =  0;

uint8_t currentAGCAtt      =  0;
uint8_t bwIdxSSB;
uint8_t bwIdxAM;
uint8_t bandIdx;
uint8_t currentMode = FM;
uint8_t previousMode;
uint8_t encoderStatus;
uint16_t freqstep;
uint8_t freqstepnr = 0; //1kHz
int   freqDec = 0;
float Displayfreq      = 0;
float currentFrequency = 0;
float dpfrq            = 0;
float fact             = 1;

String BWtext;
String RDSbuttext;
String AGCgainbuttext;
String BFObuttext;
String AGCbuttext;
String MUTEbuttext;

const char *bandwidthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
const char *bandwidthAM[]  = {"6.0", "4.0", "3.0", "2.0", "1.0", "1.8", "2.5"};
const char *Keypathtext[]  = {"1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "ENTER", "Clear"};
const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM ", "SYN"};

char buffer[64]; // Useful to handle string
char buffer1[64];

const char *stationName;
char bufferStatioName[40];

char bufferFrequency[15];
char bufferVFO[15];
char bufferUnit[5];
char bufferBandName[10];
char bufferVolume[10];
char bufferAgcGain[10];
char bufferRDS[65];
char bufferAux[15];

// const int ledChannel = 0;
// const int resolution = 1;

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;
uint8_t currentBright = 4;
int brightness[] = {10,30,60,120,220};
byte b=4;
int frame=0;
int dispBut;

typedef struct // Buttons first layer
{
  const char *BrightName;
  int         BrightValue;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
} BrightTable;


BrightTable Brightness[] = {
  {"20%",  10},        // 0
  {"40%",  30},        // 1
  {"60%",  60},        // 2
  {"80%",  120},       // 3
  {"100%", 220}        // 4
};


//=======================================================   Buttons First and Third Layer   ==========================
typedef struct // Buttons first layer
{
  const char *ButtonNam;
  
  int    ButtonNum;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
  const char *ButtonNam1;
  int    ButtonNum1;      // Button location at display from 0 to 11. To move around buttons freely at third layer.
  int    XButos;          // Xoffset
  long   YButos;          // Yoffset
} Button;

int ytotoffset = 0;

//  Button table
int Xbutst  =   -80*0;               // X Start location Buttons
int Ybutst  = 88 + ytotoffset;  // Y Start location Buttons

int Xsmtr   =   0;
int Ysmtr   =  88 + ytotoffset;  // S meter

int XVolInd =   -80*0;
int YVolInd = 111 + ytotoffset;  // Volume indicator

int XFreqDispl  =   0;
int YFreqDispl  =   0 + ytotoffset;  // display

int Xbutsiz =  80;  //size of buttons first & third layer
//int Ybutsiz =  40;
int Ybutsiz =  46;

#ifdef IhaveTDisplayTFT
int Xbut0  = 0 * Xbutsiz ; int Ybut0  = 0 * Ybutsiz; // location calqualation for 12 first layer buttons
int Xbut1  = 1 * Xbutsiz ; int Ybut1  = 0 * Ybutsiz;
int Xbut2  = 2 * Xbutsiz ; int Ybut2  = 0 * Ybutsiz;
int Xbut3  = 3 * Xbutsiz ; int Ybut3  = 0 * Ybutsiz;
int Xbut4  = 4 * Xbutsiz ; int Ybut4  = 0 * Ybutsiz;
int Xbut5  = 5 * Xbutsiz ; int Ybut5  = 0 * Ybutsiz;
int Xbut6  = 6 * Xbutsiz ; int Ybut6  = 0 * Ybutsiz;
int Xbut7  = 7 * Xbutsiz ; int Ybut7  = 0 * Ybutsiz;
int Xbut8  = 8 * Xbutsiz ; int Ybut8  = 0 * Ybutsiz;
int Xbut9  = 9 * Xbutsiz ; int Ybut9  = 0 * Ybutsiz;
int Xbut10 = 10 * Xbutsiz ; int Ybut10 = 0 * Ybutsiz;
int Xbut11 = 11 * Xbutsiz ; int Ybut11 = 0 * Ybutsiz;
int Xbut12 = 12 * Xbutsiz ; int Ybut12 = 0 * Ybutsiz;
int Xbut13 = 13 * Xbutsiz ; int Ybut13 = 0 * Ybutsiz;
int Xbut14 = 14 * Xbutsiz ; int Ybut14 = 0 * Ybutsiz;
int Xbut15 = 15 * Xbutsiz ; int Ybut15 = 0 * Ybutsiz;
int Xbut16 = 16 * Xbutsiz ; int Ybut16 = 0 * Ybutsiz;
int selectedMenu = 0;
#endif

#define HAM       0
#define BFO       1
#define FREQ      2
#define AGC       3
#define MUTE      4
#define VOL       5
#define MODE      6
#define BANDW     7
#define STEP      8
#define BROAD     9
#define PRESET   10
#define SEEKUP   11
#define SEEKDN   12
#define STATUS   13
#define RDSbut   14
#define AGCset   15
#define Bright   16

#ifdef IhaveTDisplayTFT                                                 
Button bt[] = {                                                 
  { "HAM"   ,  0 , "", 0 , Xbut0  , Ybut0  }, //     |----|----|----|----|----|----|----|--...
  { "BROAD" ,  1 , "", 1 , Xbut1  , Ybut1  }, //     |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  ...
  { "FREQ"  ,  2 , "", 2 , Xbut2  , Ybut2  }, //     |----|----|----|----|----|----|----|--...
  { "BFO"   ,  3 , "", 3 , Xbut3  , Ybut3  },
  { "AGC"   ,  4 , "", 4 , Xbut4  , Ybut4  }, 
  { "BANDW" ,  5 , "", 5 , Xbut5  , Ybut5  },
  { "STEP"  ,  6 , "", 6 , Xbut6  , Ybut6  },
  { "VOL"   ,  7 , "", 7 , Xbut7  , Ybut7  },
  { "MUTE"  ,  8 , "", 8 , Xbut8  , Ybut8  },
  { "MODE"  ,  9 , "", 9 , Xbut9  , Ybut9  },
  { "PRESET", 10 , "",10 , Xbut10 , Ybut10 },
  { "SEEKUP", 11 , "",11 , Xbut11 , Ybut11 },
  { "SEEKDN", 12 , "",12 , Xbut12 , Ybut12 },
  { "STATUS", 13 , "",13 , Xbut13 , Ybut13 },
  { "RDS"   , 14 , "",14 , Xbut14 , Ybut14 },
  { "AGCset", 15 , "",15 , Xbut15 , Ybut15 },
  { "Bright", 16 , "",16 , Xbut16 , Ybut16 }
};
#endif

// You may freely move around the button (blue) position on the display to your flavour by changing the position in ButtonNum and ButtonNum1
// You have to stay in the First or Third Layer
//======================================================= End  Buttons First  and Third Layer   ======================


//======================================================= Generic Button Layer Definitions     ==========================
typedef struct // Generic Button Struct
{
  int GenButNum;          // GenButIdx
  int XGenButos;          // Xoffset
  int XGenButsr;          // X size rectang
  int XGenButnr;          // X next rectang
  int YGenButos;          // Yoffset
  int YGenButsr;          // X size rectang
  int YGenButnr;          // Y next rectang
} GenericButtonNumber;

//  Bandnumber table for the broad-bands

#ifdef IhaveTDisplayTFT
int XfGenBut = 0;
int XfGenButsize = 80;
int YfGenBut = 88;
int YfGenButsize = 46;

int temp_XfGenBut = 0;  // Temporary XfGenBut until replace all functions to the Generic Button Layer Mode
                        // After replacement, change all temp_XfGenBut to XfGenBut.

#endif
//======================================================= End Generic Button Layer Definitions ==========================

//======================================================= Tunings Steps     ===============================
typedef struct // Tuning steps
{
  int stepFreq;
  int Xstepos;          //Xoffset
  int Xstepsr;          //X size rectang
  int Xstepnr;          //Y next rectang
  int Ystepos;          //Yoffset
  int Ystepsr;          //Y size rectang
  int Ystepnr;          //Y next rectang
} Step;

//  Tuning steps table

#ifdef IhaveTDisplayTFT
  int Xfstep =  0;
  int Xfspsize = 80;
  int Yfstep = 88;
  int Yfspsize = 46;
#endif

Step sp[] = {
  { 1 , Xfstep, Xfspsize, Xfspsize*0, Yfstep, Yfspsize,  0},  // 0
  { 5 , Xfstep, Xfspsize, Xfspsize*1, Yfstep, Yfspsize,  0},  // 1
  { 9 , Xfstep, Xfspsize, Xfspsize*2, Yfstep, Yfspsize,  0},  // 2
  { 10, Xfstep, Xfspsize, Xfspsize*3, Yfstep, Yfspsize,  0}   // 3
};
int selectedStep = 0;
//======================================================= End Tunings Steps     ===============================

//======================================================= Modulation Types     ================================
typedef struct // MODULATION
{
  int Modenum;
  int Xmodos;          //Xoffset
  int Xmodsr;          //X size rectang
  int Xmodnr;          //X next rectang
  int Ymodos;          //Yoffset
  int Ymodsr;          //Y size rectang
  int Ymodnr;          //Y next rectang
} Mode;
//  Modulation table

#ifdef IhaveTDisplayTFT
  int Xfmod = 0;
  int Xfmodsize = 80;
  int Yfmod = 88;
  int Yfmodsize = 46;
#endif

Mode md[] = {
  { 0  , Xfmod, Xfmodsize, Xfmodsize*0, Yfmod, Yfmodsize, 0},
  { 1  , Xfmod, Xfmodsize, Xfmodsize*1, Yfmod, Yfmodsize, 0},
  { 2  , Xfmod, Xfmodsize, Xfmodsize*2, Yfmod, Yfmodsize, 0},
  { 3  , Xfmod, Xfmodsize, Xfmodsize*3, Yfmod, Yfmodsize, 0},
  { 4  , Xfmod, Xfmodsize, Xfmodsize*4, Yfmod, Yfmodsize, 0}
};
int selectedMode = 0;
//======================================================= End Modulation Types     ============================

//======================================================= Keypath     =========================================
typedef struct // Keypath
{
  uint16_t KeypNum;
  uint16_t Xkeypos;          //Xoffset
  uint16_t Xkeypsr;          //X size rectang
  uint16_t Xkeypnr;          //Y next rectang
  uint16_t Ykeypos;          //Yoffset
  uint16_t Ykeypsr;          //X size rectang
  uint16_t Ykeypnr;          //Y next rectang
} Keypath;

//  Keypath table

#ifdef IhaveTDisplayTFT
uint16_t Xpath = 30;
uint16_t Ypath = 30;

Keypath kp[] = {
  {  0 , Xpath,  60 ,   0 , Ypath , 60 ,   0},
  {  1 , Xpath,  60 ,  60 , Ypath , 60 ,   0},
  {  2 , Xpath,  60 , 120 , Ypath , 60 ,   0},
  {  3 , Xpath,  60 ,   0 , Ypath , 60 ,  60},
  {  4 , Xpath,  60 ,  60 , Ypath , 60 ,  60},
  {  5 , Xpath,  60 , 120 , Ypath , 60 ,  60},
  {  6 , Xpath,  60 ,   0 , Ypath , 60 , 120},
  {  7 , Xpath,  60 ,  60 , Ypath , 60 , 120},
  {  8 , Xpath,  60 , 120 , Ypath , 60 , 120},
  {  9 , Xpath,  60 ,   0 , Ypath , 60 , 180},
  { 10 , Xpath,  60 ,  60 , Ypath , 60 , 180},
  { 11 , Xpath,  60 , 120 , Ypath , 60 , 180},
};
#endif
int selectedKeypath = 0;
//======================================================= End Keypath     =====================================

//======================================================= Bandwidth AM & FM     ===============================
typedef struct // Bandwidth AM & SSB
{
  int BandWidthAM;
  int BandWidthSSB;
  int Xos;          //Xoffset
  int Xsr;          //X size rectang
  int Xnr;          //X next rectang
  int Yos;          //Yoffset
  int Ysr;          //X size rectang
  int Ynr;          //Y next rectang
} Bandwidth;

//  Bandwidth table

#ifdef IhaveTDisplayTFT
int XfBW = 0;
int XfBWsize = 80;
int YfBW = 88;
int YfBWsize = 46;
#endif

Bandwidth bw[] = {
  { 4 , 4 , XfBW, XfBWsize, XfBWsize*0, YfBW, YfBWsize,   0},
  { 5 , 5 , XfBW, XfBWsize, XfBWsize*1, YfBW, YfBWsize,   0},
  { 3 , 0 , XfBW, XfBWsize, XfBWsize*2, YfBW, YfBWsize,   0},
  { 6 , 1 , XfBW, XfBWsize, XfBWsize*3, YfBW, YfBWsize,   0},
  { 2 , 2 , XfBW, XfBWsize, XfBWsize*4, YfBW, YfBWsize,   0},
  { 1 , 3 , XfBW, XfBWsize, XfBWsize*5, YfBW, YfBWsize,   0},
  { 0 , 0 , XfBW, XfBWsize, XfBWsize*6, YfBW, YfBWsize,   0},
};
int selectedBW = 0;
//======================================================= End Bandwidth AM & FM     ===========================

//======================================================= Broad Band Definitions     ==========================
typedef struct // Broad-Band switch
{
  int BbandNum; // bandIdx
  int Xbbandos;          //Xoffset
  int Xbbandsr;          //X size rectang
  int Xbbandnr;          //X next rectang
  int Ybbandos;          //Yoffset
  int Ybbandsr;          //X size rectang
  int Ybbandnr;          //Y next rectang
} BBandnumber;

//  Bandnumber table for the broad-bands

#ifdef IhaveTDisplayTFT
int Xfbband = 0;
int Xfbsize = 80;
int Yfbband = 88;
int Yfbsize = 46;
#endif

BBandnumber bb[] = {
  {  0 , Xfbband, Xfbsize ,  80*0 ,  Yfbband , Yfbsize ,   0}, // 0
  {  1 , Xfbband, Xfbsize ,  80*1 ,  Yfbband , Yfbsize ,   0}, // 1
  {  2 , Xfbband, Xfbsize ,  80*2 ,  Yfbband , Yfbsize ,   0}, // 2
  {  6 , Xfbband, Xfbsize ,  80*3 ,  Yfbband , Yfbsize ,   0}, // 3
  {  7 , Xfbband, Xfbsize ,  80*4 ,  Yfbband , Yfbsize ,   0}, // 4
  {  9 , Xfbband, Xfbsize ,  80*5 ,  Yfbband , Yfbsize ,   0}, // 5
  { 11 , Xfbband, Xfbsize ,  80*6 ,  Yfbband , Yfbsize ,   0}, // 6
  { 13 , Xfbband, Xfbsize ,  80*7 ,  Yfbband , Yfbsize ,   0}, // 7
  { 14 , Xfbband, Xfbsize ,  80*8 ,  Yfbband , Yfbsize ,   0}, // 8
  { 16 , Xfbband, Xfbsize ,  80*9 ,  Yfbband , Yfbsize ,   0}, // 9
  { 17 , Xfbband, Xfbsize , 80*10 ,  Yfbband , Yfbsize ,   0}, //10
  { 19 , Xfbband, Xfbsize , 80*11 ,  Yfbband , Yfbsize ,   0}, //11
  { 21 , Xfbband, Xfbsize , 80*12 ,  Yfbband , Yfbsize ,   0}, //12
  { 22 , Xfbband, Xfbsize , 80*13 ,  Yfbband , Yfbsize ,   0}, //13
  { 24 , Xfbband, Xfbsize , 80*14 ,  Yfbband , Yfbsize ,   0}, //14
  { 26 , Xfbband, Xfbsize , 80*15 ,  Yfbband , Yfbsize ,   0}, //15
  { 27 , Xfbband, Xfbsize , 80*16 ,  Yfbband , Yfbsize ,   0}, //16
  { 29 , Xfbband, Xfbsize , 80*17 ,  Yfbband , Yfbsize ,   0}, //17

};
int selectedBroad = 0;
//======================================================= End Broad Band Definitions     ======================

//======================================================= Ham Band Definitions     ============================
typedef struct // Ham Band switch
{
  int BandNum; // bandIdx
  int HamBandTxt;
  int Xbandos;          //Xoffset
  int Xbandsr;          //X size rectang
  int Xbandnr;          //X next rectang
  int Ybandos;          //Yoffset
  int Ybandsr;          //Y size rectang
  int Ybandnr;          //Y next rectang
} Bandnumber;

//  Bandnumber table for the hambands

#ifdef IhaveTDisplayTFT
  int Xfband = 0;
  int Xfsize = 80;
  int Yfband = 88;
  int Yfsize = 46;
#endif

Bandnumber bn[] = {
  {  3 , 0 , Xfband, Xfsize , 80*0 , Yfband , Yfsize , 0},
  {  4 , 1 , Xfband, Xfsize , 80*1 , Yfband , Yfsize , 0},
  {  5 , 2 , Xfband, Xfsize , 80*2 , Yfband , Yfsize , 0},
  {  8 , 3 , Xfband, Xfsize , 80*3 , Yfband , Yfsize , 0},
  { 10 , 4 , Xfband, Xfsize , 80*4 , Yfband , Yfsize , 0},
  { 12 , 5 , Xfband, Xfsize , 80*5 , Yfband , Yfsize , 0},
  { 15 , 6 , Xfband, Xfsize , 80*6 , Yfband , Yfsize , 0},
  { 18 , 7 , Xfband, Xfsize , 80*7 , Yfband , Yfsize , 0},
  { 20 , 8 , Xfband, Xfsize , 80*8 , Yfband , Yfsize , 0},
  { 23 , 9 , Xfband, Xfsize , 80*9 , Yfband , Yfsize , 0},
  { 25 , 10 ,Xfband, Xfsize , 80*10, Yfband , Yfsize , 0},
  { 28 , 11 ,Xfband, Xfsize , 80*11, Yfband , Yfsize , 0}
};
int selectedHam = 0;

//======================================================= End Ham Band Definitions     ========================

//======================================================= Display Brightness      =============================
GenericButtonNumber brt[] = {
  { 0  , XfGenBut, XfGenButsize, XfGenButsize*0, YfGenBut, YfGenButsize, 0},
  { 1  , XfGenBut, XfGenButsize, XfGenButsize*1, YfGenBut, YfGenButsize, 0},
  { 2  , XfGenBut, XfGenButsize, XfGenButsize*2, YfGenBut, YfGenButsize, 0},
  { 3  , XfGenBut, XfGenButsize, XfGenButsize*3, YfGenBut, YfGenButsize, 0},
  { 4  , XfGenBut, XfGenButsize, XfGenButsize*4, YfGenBut, YfGenButsize, 0}
};
int selectedBright = 4;
//======================================================= End Display Brightness  =============================

//======================================================= THE Band Definitions     ============================
typedef struct // Band data
{
  const char *bandName; // Bandname
  uint8_t  bandType;    // Band type (FM, MW or SW)
  uint16_t prefmod;     // Pref. modulation
  uint16_t minimumFreq; // Minimum frequency of the band
  uint16_t maximumFreq; // maximum frequency of the band
  uint16_t currentFreq; // Default frequency or current frequency
  uint16_t currentStep; // Default step (increment and decrement)
  //float BFOf1;            // BFO set for f1
  //float F1b;              // Freq1 in kHz
  //float BFOf2;            // BFO set for f2
  //float F2b;              // Freq2 in kHz
} Band;

//   Band table

Band band[] = {
  {   "FM", FM_BAND_TYPE,  FM,  7600, 10800,  9890,10}, //  FM          0
  {   "OL", LW_BAND_TYPE,  AM,   153,   279,   198, 1}, //  LW          1
  {   "OM", MW_BAND_TYPE,  AM,   520,  1710,  1000,10}, //  MW          2
  { "TUDO", LW_BAND_TYPE,  AM,   150, 30000,   284, 1}, // All/LW/MW/SW 3
  {  "630", SW_BAND_TYPE, LSB,   472,   479,   475, 1}, // Ham  630M    4
  { "160M", SW_BAND_TYPE, LSB,  1800,  1910,  1899, 1}, // Ham  160M    5
  {   "OT", SW_BAND_TYPE,  AM,  1920,  3200,  2400, 5}, //      120M    6
  {  "90M", SW_BAND_TYPE,  AM,  3200,  3400,  3300, 5}, //       90M    7
  {  "80M", SW_BAND_TYPE, LSB,  3500,  3800,  3700, 1}, // Ham   80M    8
  {  "75M", SW_BAND_TYPE,  AM,  3900,  4000,  3950, 5}, //       75M    9
  {  "60M", SW_BAND_TYPE, USB,  5330,  5410,  5375, 1}, // Ham   60M   10
  {  "49M", SW_BAND_TYPE,  AM,  5900,  6200,  6000, 5}, //       49M   11
  {  "40M", SW_BAND_TYPE, LSB,  7000,  7200,  7132, 1}, // Ham   40M   12
  {  "41M", SW_BAND_TYPE,  AM,  7200,  7450,  7210, 5}, //       41M   13
  {  "31M", SW_BAND_TYPE,  AM,  9400,  9900,  9600, 5}, //       31M   14
  {  "30M", SW_BAND_TYPE, USB, 10100, 10150, 10125, 1}, // Ham   30M   15
  {  "25M", SW_BAND_TYPE,  AM, 11600, 12100, 11700, 5}, //       25M   16
  {  "22M", SW_BAND_TYPE,  AM, 13570, 13870, 13700, 5}, //       22M   17
  {  "20M", SW_BAND_TYPE, USB, 14000, 14350, 14200, 1}, // Ham   20M   18
  {  "19M", SW_BAND_TYPE,  AM, 15100, 15830, 15700, 5}, //       19M   19
  {  "17M", SW_BAND_TYPE, USB, 18068, 18168, 18100, 1}, // Ham   17M   20
  {  "16M", SW_BAND_TYPE,  AM, 17480, 17900, 17600, 5}, //       16M   21
  {  "15M", SW_BAND_TYPE,  AM, 18900, 19020, 18950, 5}, //       15M   22
  {  "15M", SW_BAND_TYPE, USB, 21000, 21450, 21350, 1}, // Ham   15M   23
  {  "13M", SW_BAND_TYPE,  AM, 21450, 21850, 21500, 5}, //       13M   24
  {  "12M", SW_BAND_TYPE, USB, 24890, 24990, 24940, 1}, // Ham   12M   25
  {  "11M", SW_BAND_TYPE,  AM, 25670, 26100, 25800, 5}, //       11M   26
  {   "PX", SW_BAND_TYPE,  AM, 26200, 27990, 27200, 1}, // CB band     27
  {  "10M", SW_BAND_TYPE, USB, 28000, 30000, 28500, 1}, // Ham   10M   28
  {   "OC", SW_BAND_TYPE,  AM,  1730, 30000, 15500, 5}  // Whole SW    29
};
//======================================================= End THE Band Definitions     ========================

//======================================================= FM Presets     ======================================
/* typedef struct // Preset data
{
  float      presetIdx;
  const char *PresetName;
} FM_Preset ;

FM_Preset preset[] = {

  10790  , "KISS",      // 00 
  9910   ,  "CBN",      // 01 
  10370  , "NOVA",      // 02 
  
};
 */
typedef struct // Preset data
{
  float presetIdx;
  const char *PresetName;
  int presetBand;
  int Xpresetos;          //Xoffset
  int Xpresetsr;          //X size rectang
  int Xpresetnr;          //X next rectang
  int Ypresetos;          //Yoffset
  int Ypresetsr;          //Y size rectang
  int Ypresetnr;          //Y next rectang
} ST_Preset;

//  Bandnumber table for the hambands

#ifdef IhaveTDisplayTFT
  int Xfpreset = 0;
  int Xfpresetsize = 80*3;
  int Yfpreset = 88;
  int Yfpresetsize = 46;
#endif

ST_Preset preset[] = {
  {  8990 , "Jovem Pan" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  {  9170 , "Educadora" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  {  9250 ,    "Cidade" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  {  9910 ,       "CBN" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  { 10030 , "JPan News" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  { 10370 ,      "Nova" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  { 10670 ,      "Band" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0},
  { 10790 ,      "Kiss" , 0 , Xfpreset, Xfpresetsize , Xfpresetsize*0 , Yfpreset , Yfpresetsize , Yfpresetsize*0}
};
int selectedPreset = 0;

//======================================================= END FM Presets     ======================================

const int lastButton = (sizeof bt / sizeof(Button)) - 1;
const int lastBand   = (sizeof band / sizeof(Band)) - 1;
const int lastHam    = (sizeof bn / sizeof(Bandnumber)) - 1;
const int lastBroad  = (sizeof bb / sizeof(BBandnumber)) - 1;
const int lastMod    = (sizeof md / sizeof(Mode)) - 1;
const int lastBW     = (sizeof bw / sizeof(Bandwidth)) - 1;
const int lastStep   = (sizeof sp / sizeof(Step)) - 1;
const int lastKPath  = (sizeof kp / sizeof(Keypath)) - 1;
const int lastPreset = (sizeof preset / sizeof (ST_Preset)) - 1;
const int lastBright = (sizeof brt / sizeof(GenericButtonNumber)) - 1;

#define offsetEEPROM       0x30
#define EEPROM_SIZE        150

struct StoreStruct {
  byte     chkDigit;
  byte     bandIdx; 
  uint16_t Freq; 
  uint8_t  currentMode;
  uint8_t  bwIdxSSB;
  uint8_t  bwIdxAM;
  uint8_t  currentStep;
  int      currentBFO;
  uint8_t  currentAGCAtt;
  uint8_t  currentVOL;
  uint8_t  currentBFOStep;
  uint8_t  RDS;
};

StoreStruct storage = {
  '#',  //First time check
    0,  //bandIdx 
 8930,  //Freq
    0,  //mode
    1,  //bwIdxSSB
    3,  //bwIdxAM
   10,  //currentStep
 -125,  //currentBFO  
    2,  //currentAGCAtt
   45,  //currentVOL 
   25,  //currentBFOStep
    1   //RDS
};

uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

TFT_eSPI tft = TFT_eSPI();

SI4735 si4735;

//=======================================================================================
void IRAM_ATTR RotaryEncFreq() {
//=======================================================================================
  // rotary encoder events
  if (!writingEeprom){
    encoderStatus = encoder.process();
    
    if (encoderStatus)
    {
      if (encoderStatus == DIR_CW)// Direction clockwise
      {
        encoderCount = 1;
      }
      else
      {
        encoderCount = -1;
      }
    }
  }
}



/**
 * Cleans the EEPROM
*/
void resetEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  for (int k = 0; k < EEPROM_SIZE; k++) {
      EEPROM.write(k, 0);
  }
  EEPROM.end();
}

// Performe a Soft Reset
void(* resetFunc) (void) = 0;



//=======================================================================================
void setup() {
//=======================================================================================
  Serial.begin(115200);
  // pinMode(TFT_BL, OUTPUT);
  pinMode(35,INPUT);
  digitalWrite(TFT_BL, displayoff);
  DISplay = true;
  DISplayAni = true;

  pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP);

  tft.init();

  #ifdef IhaveTDisplayTFT
    tft.setRotation(-1);
  #endif

  tft.fillScreen(TFT_BLACK);
  digitalWrite(TFT_BL, displayon);

  ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
  ledcAttachPin(TFT_BL, pwmLedChannelTFT);
  ledcWrite(pwmLedChannelTFT, brightness[b]);

  //tft.setRotation(0); // Rotate 0
  //tft.setRotation(1); // Rotate 90
  //tft.setRotation(2); // Rotate 180
  //tft.setRotation(3); // Rotate 270

  // Cleans the EEPROM content. 
  // if the encoder push button is pressed during the system initialization, the EEPROM will be clened.
  for (int i = 0; i < 10; i++) {
      if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
        resetEEPROM(); // Cleans the EEPROM.
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.println(F("A EEPROM FOI LIMPA!")); 
        //while(1);  // Stops the System. It is needed to turn it off. 
        tft.println(F("Restart in 4 seconds")); 
        delay(4000);
        resetFunc();
      }
      delay(100);
  } 

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.println(F("failed to initialise EEPROM"));
    // Serial.println(F("failed to initialise EEPROM"));
    while(1); 
  }

  // RESET the EEPROM
  if (EEPROM.read(offsetEEPROM) != storage.chkDigit){
    // Serial.println(F("Writing defaults...."));
    saveConfig();
  }
  loadConfig();
  printConfig();
  

  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL); //I2C for SI4735

  // Encoder pins
  pinMode(ENCODER_PIN_A , INPUT_PULLUP); //Rotary encoder Freqency/bfo/preset
  pinMode(ENCODER_PIN_B , INPUT_PULLUP);
  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), RotaryEncFreq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), RotaryEncFreq, CHANGE);

  si4735.setAudioMuteMcuPin(AUDIO_MUTE);
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);

  // Serial.println(F("I2C - ADDR"));
  // Serial.println(si4735Addr);
  // tft.fillScreen(TFT_BLUE);
  // delay(500);

  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(NULL);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println(" RADIO DSP");
  tft.setCursor(7, 40);
  tft.println(" SOFT PE0MGB-PU2CLR");
  tft.setCursor(7, 80);
  tft.println(" PCB BY THIAGO");
  tft.setCursor(7, 120);
  tft.println(" MOD BY RXAVIER");
  tft.setFreeFont(NULL);  
  delay(1000);
  if ( si4735Addr == 0 ) {
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.print("Si4735 not detected");
    while (1);
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.print(" SI4732 :  ");
    tft.println(si4735Addr, HEX);
    tft.setFreeFont(NULL);
  }
  
  //delay(300);
  tft.fillScreen(TFT_BLACK);

  // Setup the radio from last setup in EEPROM

  // Serial.println(F("Iniciando VARIAVEIS"));

  bandIdx                   = storage.bandIdx;
  band[bandIdx].currentFreq = storage.Freq;
  currentMode               = storage.currentMode;
  bwIdxSSB                  = storage.bwIdxSSB;
  bwIdxAM                   = storage.bwIdxAM;
  currentStep               = storage.currentStep;
  currentBFO                = storage.currentBFO;
  currentAGCAtt             = storage.currentAGCAtt;
  currentVOL                = storage.currentVOL;
  currentBFOStep            = storage.currentBFOStep;
  RDS                       = storage.RDS;

  // Serial.println(F("Iniciando O SI4732"));

  if (bandIdx == 0)  si4735.setup(RESET_PIN, 0); // Start in FM
  else si4735.setup(RESET_PIN, 1); // Start in AM
  if (bandIdx != 0) si4735.setAM();

  // Serial.println(F("SI4732 Iniciado"));

  freqstep = 1000;//hz
  previousBFO = -1;
  si4735.setVolume(currentVOL);
  previousVOL = currentVOL;

  BandSet();
  if (currentStep != band[bandIdx].currentStep ) band[bandIdx].currentStep = currentStep;
  currentFrequency = previousFrequency = si4735.getFrequency();
  //encBut = 600;
  encBut = HIGH;
  dispBut = HIGH;
  DrawFila();
  si4735.setSeekFmSpacing(1);        
  si4735.setSeekFmLimits(8750,10790);
  si4735.setSeekAmRssiThreshold(20);
  si4735.setSeekAmSrnThreshold(10);
  si4735.setSeekFmRssiThreshold(5);
  si4735.setSeekFmSrnThreshold(5);

  xTaskCreate(SaveInEeprom, "SaveInEeprom", 2048, NULL, 1, NULL);
 
  // Serial.println(F("Fim do Setup"));

}// end setup


/**
 * Cleans the buffer contents
 * 
 */
void cleanBuffer() {
  bufferVolume[0] = bufferAgcGain[0] = bufferFrequency[0] = bufferUnit[0] = bufferBandName[0] = bufferVFO[0] = '\0';
}


//=======================================================================================
void SaveInEeprom (void* arg)  {
//=======================================================================================  
  while(1) {     
    storage.bandIdx = bandIdx;
    storage.Freq =  band[bandIdx].currentFreq;
    storage.currentMode = currentMode;
    storage.bwIdxSSB = bwIdxSSB;
    storage.bwIdxAM = bwIdxAM;
    storage.currentStep = currentStep;
    storage.currentBFO = currentBFO;
    storage.currentAGCAtt = currentAGCAtt;
    storage.currentVOL = currentVOL;
    storage.currentBFOStep = currentBFOStep;
    storage.RDS = RDS;
    for (unsigned int t = 0; t < sizeof(storage); t++) {
      delay(1);
      if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)){
        delay(1);
        EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
      } 
    }  
    writingEeprom = true;
    EEPROM.commit();
    writingEeprom = false;
    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}

//=======================================================================================
void saveConfig() {
//=======================================================================================
  delay(10);
  for (unsigned int t = 0; t < sizeof(storage); t++) {
    if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)){
      EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
    } 
  }  
  EEPROM.commit();
}

//=======================================================================================
void loadConfig() {
//=======================================================================================  
  if (EEPROM.read(offsetEEPROM + 0) == storage.chkDigit) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(offsetEEPROM + t);
    // Serial.println("Load config done");  
  }    
}

//=======================================================================================
void printConfig() {
//=======================================================================================
  // Serial.println(sizeof(storage));
  // if (EEPROM.read(offsetEEPROM) == storage.chkDigit){
  //  for (unsigned int t = 0; t < sizeof(storage); t++)
  //    Serial.write(EEPROM.read(offsetEEPROM + t)); 
  // // Serial.println();
  //setSettings(0);
  //}
}

//=======================================================================================
void BandSet()  {
//=======================================================================================
  if (bandIdx == 0) currentMode = 0;// only mod FM in FM band
  if ((currentMode == AM) || (currentMode == FM)) ssbLoaded = false;

  if ((currentMode == LSB) || (currentMode == USB) || (currentMode == SYNC))
  {
    if (ssbLoaded == false) {
      loadSSB();
    }
  }
  useBand();
  setBandWidth();
}

//=======================================================================================
void useBand()  {
//=======================================================================================
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  cleanBuffer();
  
  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    bfoOn = false;
    si4735.setTuneFrequencyAntennaCapacitor(0);
    delay(100);
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
    si4735.setFMDeEmphasis(1);
    ssbLoaded = false;
    si4735.RdsInit();
    si4735.setRdsConfig(1, 2, 2, 2, 2);
  }
  else
  {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
      si4735.setTuneFrequencyAntennaCapacitor(0);
    } else { //SW_BAND_TYPE
      si4735.setTuneFrequencyAntennaCapacitor(1);
    }
    if (ssbLoaded)
    {
      si4735.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
      si4735.setSSBAutomaticVolumeControl(1);
      //si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB    
      //si4735.setSSBDspAfc(0);
      //si4735.setSSBAvcDivider(3);
      //si4735.setSsbSoftMuteMaxAttenuation(8); // Disable Soft Mute for SSB
      //si4735.setSBBSidebandCutoffFilter(0);
     
      
      si4735.setSSBBfo(currentBFO);     
    }
    else
    {
      si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
      //si4735.setAutomaticGainControl(1, 0);
      //si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
      bfoOn = false;
    }

  }
  delay(100);
}// end useband

//=======================================================================================
void setBandWidth()  {
//=======================================================================================
  if (currentMode == LSB || currentMode == USB)
  {
    si4735.setSSBAudioBandwidth(bwIdxSSB);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5)
      si4735.setSBBSidebandCutoffFilter(0);
    else
      si4735.setSBBSidebandCutoffFilter(1);
  }
  else if (currentMode == AM)
  {
    si4735.setBandwidth(bwIdxAM, 0);
  }
}

//=======================================================================================
void loadSSB()  {
//=======================================================================================
  // si4735.reset();
  si4735.queryLibraryId(); // Is it really necessary here? I will check it.
  si4735.patchPowerUp();
  delay(50);
  si4735.setI2CFastMode(); // Recommended
  //si4735.setI2CFastModeCustom(500000); // It is a test and may crash.
  si4735.downloadPatch(ssb_patch_content, size_content);
  si4735.setI2CStandardMode(); // goes back to default (100kHz)

  // delay(50);
  // Parameters
  // AUDIOBW - SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz;
  // SBCUTFLT SSB - side band cutoff filter for band passand low pass filter ( 0 or 1)
  // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
  // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
  // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
  // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
  if (currentMode == SYNC) {
    si4735.setSSBConfig(bwIdxSSB, 1, 3, 0, 0, 0); // SYNC MODE
    currentBFO = 0;
  }
  else  
    si4735.setSSBConfig(bwIdxSSB, 1, 0, 0, 0, 1);

  delay(25);
  ssbLoaded = true;
}

//=======================================================================================
void Freqcalq(int keyval)  {
//=======================================================================================
  if (Decipoint) {
    dpfrq = dpfrq + keyval / fact;
  }
  else Displayfreq = (Displayfreq + keyval) * 10;
  fact = fact * 10;
  tft.setFreeFont(&Serif_bold_20);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(30, 20);
  if (Decipoint) {
    tft.print((Displayfreq / 10) + dpfrq, 4);
  }
  else {
    tft.print((Displayfreq / 10) + dpfrq, 0);
    if (((Displayfreq / 10) + dpfrq) <= 30000); 
    tft.setFreeFont(NULL);
  }
}


//=======================================================================================
void Smeter() {
//=======================================================================================
  int spoint;
  if (currentMode != FM) {
    //dBuV to S point conversion HF
    if ((rssi >= 0) and (rssi <=  1)) spoint =  12;                    // S0
    if ((rssi >  1) and (rssi <=  1)) spoint =  24;                    // S1
    if ((rssi >  2) and (rssi <=  3)) spoint =  36;                    // S2
    if ((rssi >  3) and (rssi <=  4)) spoint =  48;                    // S3
    if ((rssi >  4) and (rssi <= 10)) spoint =  48+(rssi- 4)*2;        // S4
    if ((rssi > 10) and (rssi <= 16)) spoint =  60+(rssi-10)*2;        // S5
    if ((rssi > 16) and (rssi <= 22)) spoint =  72+(rssi-16)*2;        // S6
    if ((rssi > 22) and (rssi <= 28)) spoint =  84+(rssi-22)*2;        // S7
    if ((rssi > 28) and (rssi <= 34)) spoint =  96+(rssi-28)*2;        // S8
    if ((rssi > 34) and (rssi <= 44)) spoint = 108+(rssi-34)*2;        // S9
    if ((rssi > 44) and (rssi <= 54)) spoint = 124+(rssi-44)*2;        // S9 +10
    if ((rssi > 54) and (rssi <= 64)) spoint = 140+(rssi-54)*2;        // S9 +20
    if ((rssi > 64) and (rssi <= 74)) spoint = 156+(rssi-64)*2;        // S9 +30
    if ((rssi > 74) and (rssi <= 84)) spoint = 172+(rssi-74)*2;        // S9 +40
    if ((rssi > 84) and (rssi <= 94)) spoint = 188+(rssi-84)*2;        // S9 +50
    if  (rssi > 94)                   spoint = 204;                    // S9 +60
    if  (rssi > 95)                   spoint = 208;                    //>S9 +60
  }
  else
  {
    //dBuV to S point conversion FM
    if  (rssi <  1) spoint = 36;
    if ((rssi >  1) and (rssi <=  2)) spoint =  60;                    // S6
    if ((rssi >  2) and (rssi <=  8)) spoint =  84+(rssi- 2)*2;        // S7 
    if ((rssi >  8) and (rssi <= 14)) spoint =  96+(rssi- 8)*2;        // S8
    if ((rssi > 14) and (rssi <= 24)) spoint = 108+(rssi-14)*2;        // S9
    if ((rssi > 24) and (rssi <= 34)) spoint = 124+(rssi-24)*2;        // S9 +10
    if ((rssi > 34) and (rssi <= 44)) spoint = 140+(rssi-34)*2;        // S9 +20
    if ((rssi > 44) and (rssi <= 54)) spoint = 156+(rssi-44)*2;        // S9 +30
    if ((rssi > 54) and (rssi <= 64)) spoint = 172+(rssi-54)*2;        // S9 +40
    if ((rssi > 64) and (rssi <= 74)) spoint = 188+(rssi-64)*2;        // S9 +50
    if  (rssi > 74)                   spoint = 204;                    // S9 +60
    if  (rssi > 76)                   spoint = 208;                    //>S9 +60
  }
  
  tft.fillRect(Xsmtr + 30, Ysmtr + 10 , (2 + spoint), 6, TFT_GREEN);
  tft.fillRect(Xsmtr + 32 + spoint, Ysmtr + 10 , 197 - (2 + spoint), 6, TFT_DARKGREEN);
}

//=======================================================================================
void VolumeIndicator(int vol) {
//=======================================================================================
  vol = map(vol, 0, 63, 0, 197);
  tft.fillRect(XVolInd + 30, YVolInd + 10 , (2 + vol), 6, TFT_ORANGE);
  tft.fillRect(XVolInd + 32 + vol, YVolInd + 10 , 197 - (2 + vol), 6, TFT_RED);
}


//=======================================================================================
void loop() {
//=======================================================================================
 
  VolumeIndicator(si4735.getVolume());
/* 
  Serial.print("encoderCount: ");
  Serial.print(encoderCount);
  Serial.print(" / encBut: ");
  Serial.print(encBut);
  Serial.print(" / writingEeprom: ");
  Serial.print(writingEeprom);
  Serial.print(" / dispBut: ");
  Serial.println(dispBut);
 */

  // Pressed will be set true is there is a valid touch on the screen
  while (((encoderCount == 0) and (encBut == HIGH) and (dispBut == 1)) or (writingEeprom)) {  // wait loop  
    encBut = digitalRead(ENCODER_PUSH_BUTTON);
    dispBut = digitalRead(35);
    if ( DISplay and DISplayAni) {
    showtimeRSSI();
    DisplayRDS();
    }
    DispAnimation();
    Dispoff();
  }

  encoderCheck();        // Check if the encoder has moved.
  encoderButtonCheck();  // Check if encoderbutton is pressed
  // displayBrightness();    // Check if displayButton is pressed

  if (currentMode == LSB || currentMode == USB) // set BFO value in si4735
  {
    if (currentBFO != previousBFO)
    {
      previousBFO = currentBFO;
      si4735.setSSBBfo(currentBFO);
      if (bfoOn) FreqDispl();
    }
  }

//=======================================================================================
}// end loop
//=======================================================================================

//=======================================================================================
void DispAnimation()  {
//=======================================================================================
  if (((millis() - DisplayAniTime) > MIN_ELAPSED_ANIMATION_TIME) and (DISplay == true)) {
    if ( DISplayAni ) {
      DISplayAni = false;
      tft.setTextColor(TFT_WHITE,TFT_BLACK);
      tft.setTextSize(1);
      tft.fillScreen(TFT_BLACK);
      tft.setSwapBytes(true);
    }
    tft.pushImage(0, 0, animation_width, animation_height, frameNumber[frame]);
    frame++;
    if(frame>=10)
    frame=0;
    delay(80);
  }
}

//=======================================================================================
void Dispoff()  {
//=======================================================================================
  if (((millis() - DisplayOnTime) > MIN_ELAPSED_DISPL_TIME) and (DISplay == true)) {
    DISplay = false;
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(1);
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
    // Serial.println("Display off");
    ledcWrite(pwmLedChannelTFT, 0);
    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    DisplayOnTime = millis();
  }
}

//=======================================================================================
void DisplayRDS()  {
//=======================================================================================
  if ( currentMode == FM ){
      if ( currentFrequency != previousFrequency ) {
        previousFrequency = currentFrequency;
        //bufferStatioName[0] = '\0';
        //stationName = '\0';
        tft.fillRect(XFreqDispl + 60, YFreqDispl + 54, 140, 20, TFT_BLACK);  // clear RDS text
      }
      if ((RDS) and  (NewSNR >= 12)) checkRDS(); 
      else  tft.fillRect(XFreqDispl + 60, YFreqDispl + 54, 140, 20, TFT_BLACK); // clear RDS text
  }
}

//=======================================================================================
void showtimeRSSI() {
//=======================================================================================
  // Show RSSI status only if this condition has changed
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 3) // 150 * 10  = 1.5 sec refresh time RSSI
  {
    si4735.getCurrentReceivedSignalQuality();
    NewRSSI = si4735.getCurrentRSSI();
    NewSNR = si4735.getCurrentSNR();
    if (OldRSSI != NewRSSI)
    {
      OldRSSI = NewRSSI;
      showRSSI();
    }
    elapsedRSSI = millis();
  }
}

//=======================================================================================
void showRSSI() {
//=======================================================================================
  if ( currentMode == FM ) {
    sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? "STEREO" : "MONO");
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setFreeFont(&Serif_bold_10);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(tft.textWidth("STEREO"));
    tft.fillRect(XFreqDispl + 180, YFreqDispl + 10 , 50, 12, TFT_BLACK); // STEREO MONO
    tft.drawString(buffer, XFreqDispl + 200, YFreqDispl + 20);
    tft.setFreeFont(NULL);
  }
  rssi = NewRSSI;
  Smeter();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setFreeFont(&Serif_bold_10);
  tft.setTextSize(1);
  tft.fillRect(XFreqDispl + 7, YFreqDispl + 75 , 173, 10, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.drawString("RSSI = "+ String(NewRSSI) + " dBuV" , XFreqDispl + 10, YFreqDispl + 75);
  tft.setFreeFont(NULL);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setFreeFont(&Serif_bold_10);
  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("SNR = " + String(NewSNR) + " dB", XFreqDispl + 225, YFreqDispl + 75);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void encoderCheck()  {
//=======================================================================================
  if (encoderCount != 0)
  {
    int mainpurp = 1;
    DisplayOnTime = millis();
    DisplayAniTime = millis();
    if (DISplay == false or DISplayAni == false) { //  Wake-up  Display
      ledcWrite(pwmLedChannelTFT, Brightness[selectedBright].BrightValue);
      tft.writecommand(TFT_DISPON);
      tft.writecommand(TFT_SLPOUT);
      // Serial.println("Display on");
      cleanDispl();
      DrawFila();
      showtimeRSSI();
      DisplayRDS();
      DISplay = true;
      DISplayAni = true;
      mainpurp = 0;
    }
    
    if (bfoOn)  {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      mainpurp = 0;
    } 

/*     if (AGCgainbut) {     // AGC gain control
      currentAGCgain = (encoderCount == 1) ? (currentAGCgain + currentAGCgainStep) : (currentAGCgain - currentAGCgainStep);
      mainpurp = 0;
    }
 */    
    if (mainpurp == 1)
    {
     
      if (encoderCount == 1) {
        si4735.frequencyUp();
      } else {
        si4735.frequencyDown();
      }
      FreqDispl();
      band[bandIdx].currentFreq = si4735.getFrequency();
    }

    encoderCount = 0;
  }
}

void displayBrightness() {
  dispBut = digitalRead(35);
  if(dispBut == 0){
    selectedBright++;
    if(selectedBright>=5)
    selectedBright=0;
    ledcWrite(pwmLedChannelTFT, Brightness[selectedBright].BrightValue);
    delay(400);
  }
  dispBut = HIGH;
}

//=======================================================================================
void encoderButtonCheck()  {
//=======================================================================================
  //Encoder button
  encBut = digitalRead(ENCODER_PUSH_BUTTON);
  if (encBut == LOW) {
    encBut = HIGH;
    DisplayOnTime = millis();
    DisplayAniTime = millis();
    delay(400);
    if (DISplay == false or DISplayAni == false) { //  Wake-up  Display
      ledcWrite(pwmLedChannelTFT, Brightness[selectedBright].BrightValue);
      tft.writecommand(TFT_DISPON);
      tft.writecommand(TFT_SLPOUT);
      // Serial.println("Display on");
      cleanDispl();
      DrawFila();
      showtimeRSSI();
      DisplayRDS();
      DISplay = true;
      DISplayAni = true;
      return;
    }
    // Serial.println("Showing First Layer Menu");
    DrawButFila();
    selectMenu();
    DrawSmeter();
    Smeter();
    DrawVolumeIndicator();
    VolumeIndicator(si4735.getVolume());
  }
}

void selectMenu()  {
  // Wait for a selection or timeout
  // Read rotary encoder
  int selectMenuTimeout = 4000;
  int selectMenuTime = millis();
  while ( millis() - selectMenuTime < selectMenuTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedMenu > 0 and selectedMenu < lastButton ) {
        Xbutst = (encoderCount == 1) ? (Xbutst - 1*Xbutsiz) : (Xbutst + 1*Xbutsiz);
        if ( Xbutst < -Xbutsiz*(lastButton - 2) ) {
          Xbutst = -Xbutsiz*(lastButton - 2);
        }
        if ( Xbutst > -Xbutsiz*0) {
          Xbutst = ( -Xbutsiz*0 );
        }
      }      
      selectedMenu = (encoderCount == 1) ? (selectedMenu + 1) : (selectedMenu - 1);
      if ( selectedMenu < 0 ) {
        selectedMenu = 0;
      } else {
        if ( selectedMenu > lastButton) {
          selectedMenu = lastButton;
        } else DrawButFila();
      }        
      selectMenuTime = millis();
      encoderCount = 0;
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
       delay(400);
      //  selectMenuTime = millis();
       switch (selectedMenu) {
         case 0:  //HamBand button
          //  Serial.println("HAM Menu Selected");
           delay(200);
           selectHam();
           return;
           break;
         case 1:  //Broad button
          //  Serial.println("BROAD Menu Selected");
           delay(200);
           selectBroad();
           return;
           break;
         case 2:  //Freq button
          //  Serial.println("FREQ Menu Selected");
           tft.setTextColor(TFT_WHITE, TFT_BLUE);
           tft.setFreeFont(&Serif_bold_15);
           tft.setTextSize(1);
           tft.setTextDatum(BC_DATUM);
           tft.setTextPadding(0);
           tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
           tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
           tft.drawString("Not Implemented Yet!", ((80*3) / 2) , (46 / 2) + 9 + 88);
           tft.setFreeFont(NULL);
           
           //============== To do - Is it make sense? ========
        /* FREQbut = true;
           Decipoint = false;
           Displayfreq = 0;
           dpfrq = 0;
           drawKeyPath(); */
           //============== End To do - Is it make sense? ========

           delay(1000);

           return;
           break;
         case 3:  //BFO button
          //  Serial.println("BFO Menu Selected");
           if (ssbLoaded) {  // SSB is on
             if (bfoOn) {
               bfoOn = false;
             }
             else {
               bfoOn = true;
             }
             //if (currentMode == FM) bfoOn = false;
             cleanDispl();  // TIDO --> Is that necessary?       
             drawBFO();
             delay(500);
             DrawDispl();
           } else {
    
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Valid only in SSB Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               
               delay(1000);
           }

           return;
           break;
         case 4:  //AGC button
          //  Serial.println("AGC Menu Selected");
           si4735.getAutomaticGainControl();
           AGCgain = 0;
           if  (si4735.isAgcEnabled()) {
             si4735.setAutomaticGainControl(1, 0);     //    disabled
           } else {
             AGCgainbut = false;
             si4735.setAutomaticGainControl(0, 0);      //   enabled
           }
           //cleanDispl();
           drawAGC();
           AGCfreqdisp();
           delay(400);
           // DrawDispl ();
           return;
           break;
         case 5:  //BANDW button
          //  Serial.println("BANDW Menu Selected");
           delay(400);
            if (currentMode != FM)  {
              selectBW();
              if (currentMode == AM) BWtext = bandwidthAM[bwIdxAM];
              else BWtext = bandwidthSSB[bwIdxSSB];
              tft.setFreeFont(&Serif_plain_10);
              tft.setTextSize(1);
              tft.setTextColor(TFT_GREEN, TFT_BLACK);
              tft.setTextPadding(tft.textWidth("FT"));
              tft.drawString("FT", XFreqDispl + 112, YFreqDispl + 17);
              tft.setTextPadding(tft.textWidth("2.2kHz"));
              tft.drawString(BWtext + "kHz", XFreqDispl + 144, YFreqDispl + 17);
              tft.setFreeFont(NULL);
            } else  {
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Not valid for FM Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               delay(1000);
           }
           return;
           break;
         case 6:  //STEP button
          //  Serial.println("STEP Menu Selected");
           if (currentMode != FM)  {
             if (bfoOn) setStep();
             else {
               selectStep();
             }
             STEPfreqdisp();             
           } else  {
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Not valid for FM Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               delay(1000);
           }
           return;
           break;
         case 7:  //VOL button
          //  Serial.println("Vol Menu Selected");
           DrawSmeter();
           Smeter();
           DrawVolumeIndicator();
           VolumeIndicator(si4735.getVolume());
           while ( millis() - selectMenuTime < selectMenuTimeout ) {
             if ( encoderCount != 0 ) {
                currentVOL = (encoderCount == 1) ? (currentVOL + 1) : (currentVOL - 1);
                if (currentVOL > MaxVOL) currentVOL = MaxVOL;
                if (currentVOL < MinVOL) currentVOL = MinVOL;
                previousVOL = currentVOL;
                if (Mutestat) {
                  Mutestat = false;
                  si4735.setAudioMute(audioMuteOff);
                  DrawVolumeIndicator();
                }                  

                si4735.setVolume(currentVOL);
                VolumeIndicator(si4735.getVolume());
                selectMenuTime = millis();
                encoderCount = 0;
             }
             if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
                encoderCount = 0;
                delay(400);
                return;
             }                            
             // DrawVolumeIndicator();
           }
           encoderCount = 0;
           return;
           break;
         case 8:  //MUTE button
          //  Serial.println("MUTE Menu Selected");
           delay(200);
           if (Mutestat == false)  {
             Mutestat = true;
           }
           else  {
             Mutestat = false;
           }
           drawMUTE();
           delay(400);
           return;
           break;
         case 9:  //MODE button
          //  Serial.println("MODE Menu Selected");
            if (currentMode != FM)  {
              delay(400);// Mode
              selectMode();
            } else  {
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Not valid for FM Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               delay(1000);
           }
           return;
           break;
         case 10:  //PRESET button
          //  Serial.println("PRESET Menu Selected");
           delay(200);
           selectPreset();
           FreqDispl();
           DrawSmeter();
           Smeter();
           DrawVolumeIndicator();
           VolumeIndicator(si4735.getVolume());
           return;
           break;
         case 11:  // SEEKUP button
          //  Serial.println("SEEKUP Menu Selected");
           delay(200);
           SEEK = true;
           if ((currentMode != LSB) and (currentMode != USB))   {
             if (currentMode != FM) {     // No FM
               if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
                 si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
                 si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
               } 
               else {
                 bandIdx = 29;// all sw
                 si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
                 si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  
               }
             }
             si4735.frequencyUp();
             si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_UP);
             // si4735.seekStationProgress(SeekFreq,1);// 1 is up
             delay(300);
             currentFrequency = si4735.getFrequency();
             band[bandIdx].currentFreq = currentFrequency ;
             if (currentFrequency != previousFrequency)
             {
               previousFrequency = currentFrequency;
               // DrawDispl();
               delay(300); 
             }
           } else  {
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Not valid for SSB Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               delay(1000);
           }
           // cleanDispl();
           // showFrequency();
           SEEK = false;
           return;
           break;
         case 12:  //SEEKDN button
          //  Serial.println("SEEKDN Menu Selected");
            delay(200);
            SEEK = true;

            if ((currentMode != LSB) and (currentMode != USB))   {
              if (currentMode != FM) {     // No FM
                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                } else {
                  bandIdx = 29;// all sw
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  
                }
              }
             si4735.frequencyDown();
             // si4735.seekStationProgress(SeekFreq,0);
             si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_DOWN);
             delay(300);
             currentFrequency = si4735.getFrequency();
             band[bandIdx].currentFreq = currentFrequency ;
             if (currentFrequency != previousFrequency)
             {
               previousFrequency = currentFrequency;
               // DrawDispl();
               delay(300); 
             }
           } else  {
               tft.setTextColor(TFT_WHITE, TFT_BLUE);
               tft.setFreeFont(&Serif_bold_15);
               tft.setTextSize(1);
               tft.setTextDatum(BC_DATUM);
               tft.setTextPadding(0);
               tft.fillRect(0, 88 , 80*3 , 46, TFT_DARKGREY);
               tft.fillRect(1 , 89, (80*3) - 2, 46 - 2 , TFT_BLUE);
               tft.drawString("Not valid for SSB Mode!", ((80*3) / 2) , (46 / 2) + 9 + 88);
               tft.setFreeFont(NULL);
               delay(1000);
           }
           // cleanDispl();
           // showFrequency();
           SEEK = false;
           return;
           break;
         case 13:  //STATUS button
          //  Serial.println("STATUS Menu Selected");
           delay(200);
           subrstatus();
           delay(200);
           cleanDispl();
           DrawFila();
           showtimeRSSI();
           DisplayRDS();
           // Dispoff();
           return;
           break;
         case 14:  //RDS button
          //  Serial.println("RDS Menu Selected");
           delay(200);
           if (RDS) RDS = false;
           else RDS = true;
           DrawRDSbut();
           delay(400);
           return;           
           break;
         case 15:  //AGCset button
           selectAGC();
           return;
           break;
         case 16:  //Brightness button
           selectBright();
           DrawSmeter();
           Smeter();
           DrawVolumeIndicator();
           VolumeIndicator(si4735.getVolume());
           return;
           break;
         /* default:
           Serial.println("Invalid option!"); */

       }

    }    
  }
  return;
}

void selectHam()  {
  int selectHamTimeout = 4000;
  int selectHamTime = millis();

  if (selectedHam == 0){
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr), (bn[0].Ybandos) + (bn[0].Ybandnr), (bn[0].Xbandsr) , (bn[0].Ybandsr), TFT_RED);    
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr + 3), (bn[0].Ybandos) + (bn[0].Ybandnr + 3), (bn[0].Xbandsr - 6) , (bn[0].Ybandsr - 6), TFT_BLUE);
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr), (bn[1].Ybandos) + (bn[1].Ybandnr), (bn[1].Xbandsr) , (bn[1].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr + 1), (bn[1].Ybandos) + (bn[1].Ybandnr + 1), (bn[1].Xbandsr - 2) , (bn[1].Ybandsr - 2), TFT_BLUE);
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr), (bn[2].Ybandos) + (bn[2].Ybandnr), (bn[2].Xbandsr) , (bn[2].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr + 1), (bn[2].Ybandos) + (bn[2].Ybandnr + 1), (bn[2].Xbandsr - 2) , (bn[2].Ybandsr - 2), TFT_BLUE);
  } else if (selectedBroad == lastHam){
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr), (bn[0].Ybandos) + (bn[0].Ybandnr), (bn[0].Xbandsr) , (bn[0].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr + 1), (bn[0].Ybandos) + (bn[0].Ybandnr + 1), (bn[0].Xbandsr - 2) , (bn[0].Ybandsr - 2), TFT_BLUE);
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr), (bn[1].Ybandos) + (bn[1].Ybandnr), (bn[1].Xbandsr) , (bn[1].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr + 1), (bn[1].Ybandos) + (bn[1].Ybandnr + 1), (bn[1].Xbandsr - 2) , (bn[1].Ybandsr - 2), TFT_BLUE);
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr), (bn[2].Ybandos) + (bn[2].Ybandnr), (bn[2].Xbandsr) , (bn[2].Ybandsr), TFT_RED);    
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr + 3), (bn[2].Ybandos) + (bn[2].Ybandnr + 3), (bn[2].Xbandsr - 6) , (bn[2].Ybandsr - 6), TFT_BLUE);
  } else {
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr), (bn[0].Ybandos) + (bn[0].Ybandnr), (bn[0].Xbandsr) , (bn[0].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[0].Xbandos + bn[0].Xbandnr + 1), (bn[0].Ybandos) + (bn[0].Ybandnr + 1), (bn[0].Xbandsr - 2) , (bn[0].Ybandsr - 2), TFT_BLUE);
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr), (bn[1].Ybandos) + (bn[1].Ybandnr), (bn[1].Xbandsr) , (bn[1].Ybandsr), TFT_RED);    
    tft.fillRect((bn[1].Xbandos + bn[1].Xbandnr + 3), (bn[1].Ybandos) + (bn[1].Ybandnr + 3), (bn[1].Xbandsr - 6) , (bn[1].Ybandsr - 6), TFT_BLUE);
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr), (bn[2].Ybandos) + (bn[2].Ybandnr), (bn[2].Xbandsr) , (bn[2].Ybandsr), TFT_DARKGREY);    
    tft.fillRect((bn[2].Xbandos + bn[2].Xbandnr + 1), (bn[2].Ybandos) + (bn[2].Ybandnr + 1), (bn[2].Xbandsr - 2) , (bn[2].Ybandsr - 2), TFT_BLUE);
  }

  HamBandlist();
  
  while ( millis() - selectHamTime < selectHamTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedHam > 0 and selectedHam < lastHam ) {
        Xfband = (encoderCount == 1) ? (Xfband - 1*80) : (Xfband + 1*80);
        if ( Xfband < -80*(lastHam - 2) ) {
          Xfband = -80*(lastHam - 2);
        }
        if ( Xfband > -80*0) {
          Xfband = ( -80*0 );
        }
      }      
      selectedHam = (encoderCount == 1) ? (selectedHam + 1) : (selectedHam - 1);
      if ( selectedHam < 0 ) {
        selectedHam = 0;
      } else {
        if ( selectedHam > lastHam) {
          selectedHam = lastHam;
        } else HamBandlist();
      }
      selectHamTime = millis();
      encoderCount = 0;
      
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectHamTime = millis();
      bandIdx = bn[selectedHam].BandNum;
      if (ssbLoaded == false) {
        si4735.setAM();
        delay(50);
      }
      currentMode = band[bandIdx].prefmod;
      bwIdxSSB = 1;
      BandSet();
      DrawFila();
      return;
    
    }    
  }
  return;
}

void selectBroad()  {
  int selectBroadTimeout = 4000;
  int selectBroadTime = millis();
  
  if (selectedBroad == 0){
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr), (bb[0].Ybbandos) + (bb[0].Ybbandnr), (bb[0].Xbbandsr) , (bb[0].Ybbandsr), TFT_RED);    
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr + 3), (bb[0].Ybbandos) + (bb[0].Ybbandnr + 3), (bb[0].Xbbandsr - 6) , (bb[0].Ybbandsr - 6), TFT_BLUE);
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr), (bb[1].Ybbandos) + (bb[1].Ybbandnr), (bb[1].Xbbandsr) , (bb[1].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr + 1), (bb[1].Ybbandos) + (bb[1].Ybbandnr + 1), (bb[1].Xbbandsr - 2) , (bb[1].Ybbandsr - 2), TFT_BLUE);
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr), (bb[2].Ybbandos) + (bb[2].Ybbandnr), (bb[2].Xbbandsr) , (bb[2].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr + 1), (bb[2].Ybbandos) + (bb[2].Ybbandnr + 1), (bb[2].Xbbandsr - 2) , (bb[2].Ybbandsr - 2), TFT_BLUE);
  } else if (selectedBroad == lastBroad){
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr), (bb[0].Ybbandos) + (bb[0].Ybbandnr), (bb[0].Xbbandsr) , (bb[0].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr + 1), (bb[0].Ybbandos) + (bb[0].Ybbandnr + 1), (bb[0].Xbbandsr - 2) , (bb[0].Ybbandsr - 2), TFT_BLUE);
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr), (bb[1].Ybbandos) + (bb[1].Ybbandnr), (bb[1].Xbbandsr) , (bb[1].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr + 1), (bb[1].Ybbandos) + (bb[1].Ybbandnr + 1), (bb[1].Xbbandsr - 2) , (bb[1].Ybbandsr - 2), TFT_BLUE);
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr), (bb[2].Ybbandos) + (bb[2].Ybbandnr), (bb[2].Xbbandsr) , (bb[2].Ybbandsr), TFT_RED);    
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr + 3), (bb[2].Ybbandos) + (bb[2].Ybbandnr + 3), (bb[2].Xbbandsr - 6) , (bb[2].Ybbandsr - 6), TFT_BLUE);
  } else {
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr), (bb[0].Ybbandos) + (bb[0].Ybbandnr), (bb[0].Xbbandsr) , (bb[0].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[0].Xbbandos + bb[0].Xbbandnr + 1), (bb[0].Ybbandos) + (bb[0].Ybbandnr + 1), (bb[0].Xbbandsr - 2) , (bb[0].Ybbandsr - 2), TFT_BLUE);
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr), (bb[1].Ybbandos) + (bb[1].Ybbandnr), (bb[1].Xbbandsr) , (bb[1].Ybbandsr), TFT_RED);    
    tft.fillRect((bb[1].Xbbandos + bb[1].Xbbandnr + 3), (bb[1].Ybbandos) + (bb[1].Ybbandnr + 3), (bb[1].Xbbandsr - 6) , (bb[1].Ybbandsr - 6), TFT_BLUE);
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr), (bb[2].Ybbandos) + (bb[2].Ybbandnr), (bb[2].Xbbandsr) , (bb[2].Ybbandsr), TFT_DARKGREY);    
    tft.fillRect((bb[2].Xbbandos + bb[2].Xbbandnr + 1), (bb[2].Ybbandos) + (bb[2].Ybbandnr + 1), (bb[2].Xbbandsr - 2) , (bb[2].Ybbandsr - 2), TFT_BLUE);
  }

  BroadBandlist();

  while ( millis() - selectBroadTime < selectBroadTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedBroad > 0 and selectedBroad < lastBroad ) {
        Xfbband = (encoderCount == 1) ? (Xfbband - 1*80) : (Xfbband + 1*80);
        if ( Xfbband < -80*(lastBroad - 2) ) {
          Xfbband = -80*(lastBroad - 2);
        }
        if ( Xfbband > -80*0) {
          Xfbband = ( -80*0 );
        }
      }      
      selectedBroad = (encoderCount == 1) ? (selectedBroad + 1) : (selectedBroad - 1);
      if ( selectedBroad < 0 ) {
        selectedBroad = 0;
      } else {
        if ( selectedBroad > lastBroad) {
          selectedBroad = lastBroad;
        } else BroadBandlist();
      }
      selectBroadTime = millis();
      encoderCount = 0;
      
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectBroadTime = millis();

      bandIdx = bb[selectedBroad].BbandNum;
      if ((bandIdx == 0) and (currentAGCgain >=28)) currentAGCgain = previousAGCgain = 26; // currentAGCgain in FM max. 26
      si4735.setAM();
      delay(50);
      currentMode = band[bandIdx].prefmod;
      bwIdxAM =  3;
      BandSet();
      DrawFila(); //Draw first layer
      return;
    }    
  }
  return;
}

void selectStep()  {
  int selectStepTimeout = 4000;
  int selectStepTime = millis();
  
  if (selectedStep == 0){
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr), (sp[0].Ystepos) + (sp[0].Ystepnr), (sp[0].Xstepsr) , (sp[0].Ystepsr), TFT_RED);    
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr + 3), (sp[0].Ystepos) + (sp[0].Ystepnr + 3), (sp[0].Xstepsr - 6) , (sp[0].Ystepsr - 6), TFT_BLUE);
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr), (sp[1].Ystepos) + (sp[1].Ystepnr), (sp[1].Xstepsr) , (sp[1].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr + 1), (sp[1].Ystepos) + (sp[1].Ystepnr + 1), (sp[1].Xstepsr - 2) , (sp[1].Ystepsr - 2), TFT_BLUE);
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr), (sp[2].Ystepos) + (sp[2].Ystepnr), (sp[2].Xstepsr) , (sp[2].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr + 1), (sp[2].Ystepos) + (sp[2].Ystepnr + 1), (sp[2].Xstepsr - 2) , (sp[2].Ystepsr - 2), TFT_BLUE);
  } else if (selectedStep == lastStep){
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr), (sp[0].Ystepos) + (sp[0].Ystepnr), (sp[0].Xstepsr) , (sp[0].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr + 1), (sp[0].Ystepos) + (sp[0].Ystepnr + 1), (sp[0].Xstepsr - 2) , (sp[0].Ystepsr - 2), TFT_BLUE);
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr), (sp[1].Ystepos) + (sp[1].Ystepnr), (sp[1].Xstepsr) , (sp[1].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr + 1), (sp[1].Ystepos) + (sp[1].Ystepnr + 1), (sp[1].Xstepsr - 2) , (sp[1].Ystepsr - 2), TFT_BLUE);
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr), (sp[2].Ystepos) + (sp[2].Ystepnr), (sp[2].Xstepsr) , (sp[2].Ystepsr), TFT_RED);    
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr + 3), (sp[2].Ystepos) + (sp[2].Ystepnr + 3), (sp[2].Xstepsr - 6) , (sp[2].Ystepsr - 6), TFT_BLUE);
  } else {
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr), (sp[0].Ystepos) + (sp[0].Ystepnr), (sp[0].Xstepsr) , (sp[0].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[0].Xstepos + sp[0].Xstepnr + 1), (sp[0].Ystepos) + (sp[0].Ystepnr + 1), (sp[0].Xstepsr - 2) , (sp[0].Ystepsr - 2), TFT_BLUE);
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr), (sp[1].Ystepos) + (sp[1].Ystepnr), (sp[1].Xstepsr) , (sp[1].Ystepsr), TFT_RED);    
    tft.fillRect((sp[1].Xstepos + sp[1].Xstepnr + 3), (sp[1].Ystepos) + (sp[1].Ystepnr + 3), (sp[1].Xstepsr - 6) , (sp[1].Ystepsr - 6), TFT_BLUE);
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr), (sp[2].Ystepos) + (sp[2].Ystepnr), (sp[2].Xstepsr) , (sp[2].Ystepsr), TFT_DARKGREY);    
    tft.fillRect((sp[2].Xstepos + sp[2].Xstepnr + 1), (sp[2].Ystepos) + (sp[2].Ystepnr + 1), (sp[2].Xstepsr - 2) , (sp[2].Ystepsr - 2), TFT_BLUE);
  }

  Steplist();

  while ( millis() - selectStepTime < selectStepTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedStep > 0 and selectedStep < lastStep ) {
        Xfstep = (encoderCount == 1) ? (Xfstep - 1*80) : (Xfstep + 1*80);
        if ( Xfstep < -80*(lastStep - 2) ) {
          Xfstep = -80*(lastStep - 2);
        }
        if ( Xfstep > -80*0) {
          Xfstep = ( -80*0 );
        }
      }      
      selectedStep = (encoderCount == 1) ? (selectedStep + 1) : (selectedStep - 1);
      if ( selectedStep < 0 ) {
        selectedStep = 0;
      } else {
        if ( selectedStep > lastStep) {
          selectedStep = lastStep;
        } else Steplist();
      }
      selectStepTime = millis();
      encoderCount = 0;
      
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectStepTime = millis();
      currentStep = sp[selectedStep].stepFreq;
      setStep();
      return;
    }    
  }
  return;
}

//=======================================================================================
void setStep()  {
//=======================================================================================
  // This command should work only for SSB mode
  if (bfoOn && (currentMode == LSB || currentMode == USB))
  {
    currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
    BFOfreqdisp();
  }
  else
  {
    si4735.setFrequencyStep(currentStep);
    if (currentMode != FM)  {
      band[bandIdx].currentStep = currentStep;
    }
  }
  DrawSmeter();
  Smeter();
  DrawVolumeIndicator();
  VolumeIndicator(si4735.getVolume());
}

void selectMode() {
  int selectModeTimeout = 4000;
  int selectModeTime = millis();
  
  if (selectedMode == 0){
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr), (md[0].Ymodos) + (md[0].Ymodnr), (md[0].Xmodsr) , (md[0].Ymodsr), TFT_RED);    
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr + 3), (md[0].Ymodos) + (md[0].Ymodnr + 3), (md[0].Xmodsr - 6) , (md[0].Ymodsr - 6), TFT_BLUE);
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr), (md[1].Ymodos) + (md[1].Ymodnr), (md[1].Xmodsr) , (md[1].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr + 1), (md[1].Ymodos) + (md[1].Ymodnr + 1), (md[1].Xmodsr - 2) , (md[1].Ymodsr - 2), TFT_BLUE);
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr), (md[2].Ymodos) + (md[2].Ymodnr), (md[2].Xmodsr) , (md[2].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr + 1), (md[2].Ymodos) + (md[2].Ymodnr + 1), (md[2].Xmodsr - 2) , (md[2].Ymodsr - 2), TFT_BLUE);
  } else if (selectedMode == lastMod){
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr), (md[0].Ymodos) + (md[0].Ymodnr), (md[0].Xmodsr) , (md[0].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr + 1), (md[0].Ymodos) + (md[0].Ymodnr + 1), (md[0].Xmodsr - 2) , (md[0].Ymodsr - 2), TFT_BLUE);
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr), (md[1].Ymodos) + (md[1].Ymodnr), (md[1].Xmodsr) , (md[1].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr + 1), (md[1].Ymodos) + (md[1].Ymodnr + 1), (md[1].Xmodsr - 2) , (md[1].Ymodsr - 2), TFT_BLUE);
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr), (md[2].Ymodos) + (md[2].Ymodnr), (md[2].Xmodsr) , (md[2].Ymodsr), TFT_RED);    
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr + 3), (md[2].Ymodos) + (md[2].Ymodnr + 3), (md[2].Xmodsr - 6) , (md[2].Ymodsr - 6), TFT_BLUE);
  } else {
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr), (md[0].Ymodos) + (md[0].Ymodnr), (md[0].Xmodsr) , (md[0].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[0].Xmodos + md[0].Xmodnr + 1), (md[0].Ymodos) + (md[0].Ymodnr + 1), (md[0].Xmodsr - 2) , (md[0].Ymodsr - 2), TFT_BLUE);
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr), (md[1].Ymodos) + (md[1].Ymodnr), (md[1].Xmodsr) , (md[1].Ymodsr), TFT_RED);    
    tft.fillRect((md[1].Xmodos + md[1].Xmodnr + 3), (md[1].Ymodos) + (md[1].Ymodnr + 3), (md[1].Xmodsr - 6) , (md[1].Ymodsr - 6), TFT_BLUE);
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr), (md[2].Ymodos) + (md[2].Ymodnr), (md[2].Xmodsr) , (md[2].Ymodsr), TFT_DARKGREY);    
    tft.fillRect((md[2].Xmodos + md[2].Xmodnr + 1), (md[2].Ymodos) + (md[2].Ymodnr + 1), (md[2].Xmodsr - 2) , (md[2].Ymodsr - 2), TFT_BLUE);
  }

  Modelist();

  while ( millis() - selectModeTime < selectModeTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedMode > 0 and selectedMode < lastMod ) {
        Xfmod = (encoderCount == 1) ? (Xfmod - 1*80) : (Xfmod + 1*80);
        if ( Xfmod < -80*(lastMod - 2) ) {
          Xfmod = -80*(lastMod - 2);
        }
        if ( Xfmod > -80*0) {
          Xfmod = ( -80*0 );
        }
      }      
      selectedMode = (encoderCount == 1) ? (selectedMode + 1) : (selectedMode - 1);
      if ( selectedMode < 0 ) {
        selectedMode = 0;
      } else {
        if ( selectedMode > lastMod) {
          selectedMode = lastMod;
        } else Modelist();
      }
      selectModeTime = millis();
      encoderCount = 0;
      
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectModeTime = millis();
      currentMode = md[selectedMode].Modenum;
      BandSet();
      DrawFila();
      return;
    }    
  }
  return;
}

void selectBW() {
  int selectBWTimeout = 4000;
  int selectBWTime = millis();

  if ( currentMode == AM) nrbox = lastBW;
  else nrbox = lastBW - 1;

  if (selectedBW == 0){
    tft.fillRect((bw[0].Xos + bw[0].Xnr), (bw[0].Yos) + (bw[0].Ynr), (bw[0].Xsr) , (bw[0].Ysr), TFT_RED);    
    tft.fillRect((bw[0].Xos + bw[0].Xnr + 3), (bw[0].Yos) + (bw[0].Ynr + 3), (bw[0].Xsr - 6) , (bw[0].Ysr - 6), TFT_BLUE);
    tft.fillRect((bw[1].Xos + bw[1].Xnr), (bw[1].Yos) + (bw[1].Ynr), (bw[1].Xsr) , (bw[1].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[1].Xos + bw[1].Xnr + 1), (bw[1].Yos) + (bw[1].Ynr + 1), (bw[1].Xsr - 2) , (bw[1].Ysr - 2), TFT_BLUE);
    tft.fillRect((bw[2].Xos + bw[2].Xnr), (bw[2].Yos) + (bw[2].Ynr), (bw[2].Xsr) , (bw[2].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[2].Xos + bw[2].Xnr + 1), (bw[2].Yos) + (bw[2].Ynr + 1), (bw[2].Xsr - 2) , (bw[2].Ysr - 2), TFT_BLUE);
  } else if (selectedBW == lastBW){
    tft.fillRect((bw[0].Xos + bw[0].Xnr), (bw[0].Yos) + (bw[0].Ynr), (bw[0].Xsr) , (bw[0].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[0].Xos + bw[0].Xnr + 1), (bw[0].Yos) + (bw[0].Ynr + 1), (bw[0].Xsr - 2) , (bw[0].Ysr - 2), TFT_BLUE);
    tft.fillRect((bw[1].Xos + bw[1].Xnr), (bw[1].Yos) + (bw[1].Ynr), (bw[1].Xsr) , (bw[1].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[1].Xos + bw[1].Xnr + 1), (bw[1].Yos) + (bw[1].Ynr + 1), (bw[1].Xsr - 2) , (bw[1].Ysr - 2), TFT_BLUE);
    tft.fillRect((bw[2].Xos + bw[2].Xnr), (bw[2].Yos) + (bw[2].Ynr), (bw[2].Xsr) , (bw[2].Ysr), TFT_RED);    
    tft.fillRect((bw[2].Xos + bw[2].Xnr + 3), (bw[2].Yos) + (bw[2].Ynr + 3), (bw[2].Xsr - 6) , (bw[2].Ysr - 6), TFT_BLUE);
  } else {
    tft.fillRect((bw[0].Xos + bw[0].Xnr), (bw[0].Yos) + (bw[0].Ynr), (bw[0].Xsr) , (bw[0].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[0].Xos + bw[0].Xnr + 1), (bw[0].Yos) + (bw[0].Ynr + 1), (bw[0].Xsr - 2) , (bw[0].Ysr - 2), TFT_BLUE);
    tft.fillRect((bw[1].Xos + bw[1].Xnr), (bw[1].Yos) + (bw[1].Ynr), (bw[1].Xsr) , (bw[1].Ysr), TFT_RED);    
    tft.fillRect((bw[1].Xos + bw[1].Xnr + 3), (bw[1].Yos) + (bw[1].Ynr + 3), (bw[1].Xsr - 6) , (bw[1].Ysr - 6), TFT_BLUE);
    tft.fillRect((bw[2].Xos + bw[2].Xnr), (bw[2].Yos) + (bw[2].Ynr), (bw[2].Xsr) , (bw[2].Ysr), TFT_DARKGREY);    
    tft.fillRect((bw[2].Xos + bw[2].Xnr + 1), (bw[2].Yos) + (bw[2].Ynr + 1), (bw[2].Xsr - 2) , (bw[2].Ysr - 2), TFT_BLUE);
  }

  BWList();

  while ( millis() - selectBWTime < selectBWTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedBW > 0 and selectedBW < nrbox ) {
        XfBW = (encoderCount == 1) ? (XfBW - 1*80) : (XfBW + 1*80);
        if ( XfBW < -80*(nrbox - 2) ) {
          XfBW = -80*(nrbox - 2);
        }
        if ( XfBW > -80*0) {
          XfBW = ( -80*0 );
        }
      }      
      selectedBW = (encoderCount == 1) ? (selectedBW + 1) : (selectedBW - 1);
      if ( selectedBW < 0 ) {
        selectedBW = 0;
      } else {
        if ( selectedBW > nrbox) {
          selectedBW = nrbox;
        } else BWList();
      }
      selectBWTime = millis();
      encoderCount = 0;
      
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectBWTime = millis();
      if ( currentMode == AM) {
        bwIdxAM = bw[selectedBW].BandWidthAM;
      }
      else {
        bwIdxSSB = bw[selectedBW].BandWidthSSB;
        }
      BandSet();
      return;
    }    
  }
  return;
}


void selectPreset() {
  int selectPresetTimeout = 4000;
  int selectPresetTime = millis();
  
  tft.fillRect((preset[0].Xpresetos + preset[0].Xpresetnr) + Xfpreset, (preset[0].Ypresetos) + (preset[0].Ypresetnr), (preset[0].Xpresetsr) , (preset[0].Ypresetsr), TFT_RED);    
  tft.fillRect((preset[0].Xpresetos + preset[0].Xpresetnr + 3) + Xfpreset, (preset[0].Ypresetos) + (preset[0].Ypresetnr + 3), (preset[0].Xpresetsr - 6) , (preset[0].Ypresetsr - 6), TFT_BLUE);

  Presetlist();

  while ( millis() - selectPresetTime < selectPresetTimeout ) {
    if ( encoderCount != 0 ) {
      if ( selectedPreset > 0 and selectedPreset < lastPreset ) {
        Yfpreset = (encoderCount == 1) ? (Yfpreset - 1*Yfpresetsize) : (Yfpreset + 1*Yfpresetsize);
        if ( Yfpreset < -Yfpresetsize*(lastPreset - 2) ) {
          Yfpreset = -Yfpresetsize*(lastPreset - 2);
        }
        if ( Yfpreset > -Yfpresetsize*0) {
          Yfpreset = ( -Yfpresetsize*0 );
        }
      }      
      selectedPreset = (encoderCount == 1) ? (selectedPreset + 1) : (selectedPreset - 1);
      if ( selectedPreset < 0 ) {
        selectedPreset = 0;
      } else {
        if ( selectedPreset > lastPreset) {
          selectedPreset = lastPreset;
        } else Presetlist();
      }
      selectPresetTime = millis();
      encoderCount = 0;
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectPresetTime = millis();
      currentPRES = preset[selectedPreset].presetIdx;
      if (currentPRES != previousPRES or !si4735.isCurrentTuneFM() )
      {
        si4735.getCurrentReceivedSignalQuality();
        if (si4735.isCurrentTuneFM() == false) {
          bandIdx = 0;
          BandSet();
          DrawFila();
        }
        previousPRES = currentPRES;
        bandIdx = 0;
        si4735.setFrequency((preset[selectedPreset].presetIdx));
        band[bandIdx].currentFreq = si4735.getFrequency();
      }

      // BandSet();
      return;
    }    
  }
  return;
}

void selectAGC() {
  int selectAGCTimeout = 4000;
  int selectAGCTime = millis();

  bfoOn = false; // only AGC function at the rotory encoder
  AGCgainbut = true;
  si4735.getAutomaticGainControl();
  DrawDispl();
  FreqDispl();

  while ( millis() - selectAGCTime < selectAGCTimeout ) {
    if ( encoderCount != 0 ) {
      currentAGCgain = (encoderCount == 1) ? (currentAGCgain + currentAGCgainStep) : (currentAGCgain - currentAGCgainStep);
      AGCgain = 1;
      tft.setCursor(0, 20);
      if (si4735.isCurrentTuneFM())  MaxAGCgain = MaxAGCgainFM;
      else MaxAGCgain = MaxAGCgainAM;
  
      if (currentAGCgain > MaxAGCgain) currentAGCgain = MaxAGCgain;
      if (currentAGCgain < MinAGCgain) currentAGCgain = MinAGCgain;
  
      previousAGCgain = currentAGCgain;
      si4735.setAutomaticGainControl(1,currentAGCgain);
      // DrawDispl();
      FreqDispl();
      // DrawAGCgainbut();
      selectAGCTime = millis();
      encoderCount = 0;
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectAGCTime = millis() - selectAGCTimeout;
    }    
  }
  AGCgainbut = false;
  cleanDispl();
  DrawFila();
  return;
}

void selectBright() {
  int selectBrightTimeout = 4000;
  int selectBrightTime = millis();

  if (selectedBright == 0){
    XfGenBut=0;
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr), (brt[0].YGenButos) + (brt[0].YGenButnr), (brt[0].XGenButsr) , (brt[0].YGenButsr), TFT_RED);    
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr + 3), (brt[0].YGenButos) + (brt[0].YGenButnr + 3), (brt[0].XGenButsr - 6) , (brt[0].YGenButsr - 6), TFT_BLUE);
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr), (brt[1].YGenButos) + (brt[1].YGenButnr), (brt[1].XGenButsr) , (brt[1].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr + 1), (brt[1].YGenButos) + (brt[1].YGenButnr + 1), (brt[1].XGenButsr - 2) , (brt[1].YGenButsr - 2), TFT_BLUE);
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr), (brt[2].YGenButos) + (brt[2].YGenButnr), (brt[2].XGenButsr) , (brt[2].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr + 1), (brt[2].YGenButos) + (brt[2].YGenButnr + 1), (brt[2].XGenButsr - 2) , (brt[2].YGenButsr - 2), TFT_BLUE);
  } else if (selectedBright == lastBright){
    XfGenBut=-(selectedBright-2)*XfGenButsize;
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr), (brt[0].YGenButos) + (brt[0].YGenButnr), (brt[0].XGenButsr) , (brt[0].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr + 1), (brt[0].YGenButos) + (brt[0].YGenButnr + 1), (brt[0].XGenButsr - 2) , (brt[0].YGenButsr - 2), TFT_BLUE);
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr), (brt[1].YGenButos) + (brt[1].YGenButnr), (brt[1].XGenButsr) , (brt[1].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr + 1), (brt[1].YGenButos) + (brt[1].YGenButnr + 1), (brt[1].XGenButsr - 2) , (brt[1].YGenButsr - 2), TFT_BLUE);
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr), (brt[2].YGenButos) + (brt[2].YGenButnr), (brt[2].XGenButsr) , (brt[2].YGenButsr), TFT_RED);    
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr + 3), (brt[2].YGenButos) + (brt[2].YGenButnr + 3), (brt[2].XGenButsr - 6) , (brt[2].YGenButsr - 6), TFT_BLUE);
  } else {
    XfGenBut=-(selectedBright-1)*XfGenButsize;
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr), (brt[0].YGenButos) + (brt[0].YGenButnr), (brt[0].XGenButsr) , (brt[0].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[0].XGenButos + brt[0].XGenButnr + 1), (brt[0].YGenButos) + (brt[0].YGenButnr + 1), (brt[0].XGenButsr - 2) , (brt[0].YGenButsr - 2), TFT_BLUE);
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr), (brt[1].YGenButos) + (brt[1].YGenButnr), (brt[1].XGenButsr) , (brt[1].YGenButsr), TFT_RED);    
    tft.fillRect((brt[1].XGenButos + brt[1].XGenButnr + 3), (brt[1].YGenButos) + (brt[1].YGenButnr + 3), (brt[1].XGenButsr - 6) , (brt[1].YGenButsr - 6), TFT_BLUE);
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr), (brt[2].YGenButos) + (brt[2].YGenButnr), (brt[2].XGenButsr) , (brt[2].YGenButsr), TFT_DARKGREY);    
    tft.fillRect((brt[2].XGenButos + brt[2].XGenButnr + 1), (brt[2].YGenButos) + (brt[2].YGenButnr + 1), (brt[2].XGenButsr - 2) , (brt[2].YGenButsr - 2), TFT_BLUE);
  }

  Brightlist();

  while ( millis() - selectBrightTime < selectBrightTimeout ) {
    if ( encoderCount != 0 ) {
      selectedBright = (encoderCount == 1) ? (selectedBright + 1) : (selectedBright - 1);
      if ( selectedBright < 0 ) {
        XfGenBut=0;
        selectedBright = 0;
      } else {
        if ( selectedBright > lastBright) {
          XfGenBut=-(selectedBright-2)*XfGenButsize;
          selectedBright = lastBright;
        } else {
          if ( selectedBright == 0 ) XfGenBut=0;
          else if ( selectedBright ==  lastBright ) XfGenBut=-(selectedBright-2)*XfGenButsize;
          else XfGenBut=-(selectedBright-1)*XfGenButsize;
          Brightlist();
        }
      }
      selectBrightTime = millis();
      encoderCount = 0;
      ledcWrite(pwmLedChannelTFT, Brightness[selectedBright].BrightValue);
    }
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(200);
      selectBrightTime = millis();
      ledcWrite(pwmLedChannelTFT, Brightness[selectedBright].BrightValue);
      return;
    }    
  }
  return;
}


//=======================================================================================
void DrawFila()   {// Draw of first layer
//=======================================================================================
  tft.fillScreen(TFT_BLACK);
  //DrawButFila();
  DrawDispl();
  DrawSmeter();
  DrawVolumeIndicator();
}

//=======================================================================================
void DrawButFila() { // Buttons first layer
//=======================================================================================

  for ( int n = selectedMenu-2; n <= selectedMenu + 2; n++) {
    tft.setTextColor(TFT_CYAN, TFT_BLUE);
    tft.setFreeFont(&Serif_bold_15);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(tft.textWidth("88888M"));
    if (n == selectedMenu) {
      tft.fillRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_RED);
      tft.fillRect((bt[bt[n].ButtonNum].XButos + Xbutst + 3) , (bt[bt[n].ButtonNum].YButos + Ybutst + 3), (Xbutsiz - 6) , (Ybutsiz - 6), TFT_BLUE);
      tft.drawString((bt[n].ButtonNam), ( bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 9)  ));
    } else if (n >= 0 and n <= lastButton) {
             tft.fillRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_DARKGREY);
             tft.fillRect((bt[bt[n].ButtonNum].XButos + Xbutst + 1) , (bt[bt[n].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 2) , (Ybutsiz - 2), TFT_BLUE);
             tft.drawString((bt[n].ButtonNam), ( bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 9)  ));
           }                  
    tft.setTextPadding(0);
    tft.setFreeFont(NULL);
  }

  drawAGC();
  DrawRDSbut();
  drawMUTE();
  drawBFO();
}

//=======================================================================================
void DrawVolumeIndicator()  {
//=======================================================================================
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(&Serif_bold_10);
  tft.setTextSize(1);
  tft.fillRect(XVolInd, YVolInd, 240, 25, TFT_DARKGREY);
  tft.fillRect(XVolInd + 2, YVolInd + 2, 236, 21, TFT_BLACK);
  tft.setCursor(XVolInd + 5, YVolInd + 16);
  tft.print("Vol");
  if (Mutestat) {
    tft.drawLine(XVolInd + 5,YVolInd + 16,XVolInd + 5 + 20,YVolInd + 16 - 10,TFT_RED);
    tft.drawLine(XVolInd + 5,YVolInd + 16 - 10,XVolInd + 5 + 20,YVolInd + 16,TFT_RED);
    tft.drawLine(XVolInd + 5,YVolInd + 17,XVolInd + 5 + 20,YVolInd + 17 - 10,TFT_RED);
    tft.drawLine(XVolInd + 5,YVolInd + 17 - 10,XVolInd + 5 + 20,YVolInd + 17,TFT_RED);
  }
  tft.setFreeFont(NULL);
}

//=======================================================================================
void DrawSmeter()  {
//=======================================================================================
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(&Serif_bold_10);
  tft.setTextSize(1);
  tft.fillRect(Xsmtr, Ysmtr, 240, 25, TFT_DARKGREY);
  tft.fillRect(Xsmtr + 2, Ysmtr + 2, 236, 21, TFT_BLACK);
  tft.setCursor(Xsmtr + 5, Ysmtr + 16);
  tft.print("Sig");
  tft.setFreeFont(NULL);
}

//=======================================================================================
void DrawRDSbut()  {
//=======================================================================================
  int RDSbutcol;
  int isSelected;
  int border;
  if (selectedMenu == 14) {
    isSelected = TFT_RED;
    border = 3;
  } else {
    isSelected = TFT_DARKGREY;
    border = 1;
  }    
  if (RDS) {
    RDSbutcol = TFT_RED;
    tft.setTextColor(TFT_CYAN,TFT_RED);
    RDSbuttext = "ON";
  }else{ 
    RDSbutcol = TFT_BLUE;
    tft.setTextColor(TFT_CYAN,TFT_BLUE );
    RDSbuttext = "OFF";
  }
  tft.fillRect(bt[bt[14].ButtonNum1].XButos + Xbutst, bt[bt[14].ButtonNum1].YButos + Ybutst , Xbutsiz , Ybutsiz, isSelected);
  tft.fillRect((bt[bt[14].ButtonNum1].XButos + Xbutst + border) , (bt[bt[14].ButtonNum1].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, RDSbutcol);
  if (selectedMenu == 14) tft.drawRect((bt[bt[14].ButtonNum1].XButos + Xbutst + border) , (bt[bt[14].ButtonNum1].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, RDSbutcol);
  tft.setFreeFont(&Serif_bold_15);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString("RDS", ( bt[bt[14].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[14].ButtonNum1].YButos + Ybutst  + (Ybutsiz / 2+2)));
  tft.drawString(RDSbuttext,( bt[bt[14].ButtonNum1].XButos + Xbutst + (Xbutsiz/2)),(bt[bt[14].ButtonNum1].YButos+Ybutst  + (Ybutsiz/2+17)));
  tft.setFreeFont(NULL);
}


//=======================================================================================
void drawMUTE()  {
//=======================================================================================
  int MUTEbutcol;
  int isSelected;
  int border;

 if (selectedMenu == 8) {
    isSelected = TFT_RED;
    border = 3;
  } else {
    isSelected = TFT_DARKGREY;
    border = 1;
  }      
  if (Mutestat) {
    MUTEbutcol = TFT_RED;
    MUTEbuttext = "ON";
    si4735.setAudioMute(audioMuteOn);
  } else {
    MUTEbutcol = TFT_BLUE;
    MUTEbuttext = "OFF";
    si4735.setAudioMute(audioMuteOff);
  }
  tft.fillRect(bt[bt[8].ButtonNum].XButos + Xbutst, bt[bt[8].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, isSelected);
  tft.fillRect((bt[bt[8].ButtonNum].XButos + Xbutst + border) , (bt[bt[8].ButtonNum].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, MUTEbutcol);
  if (selectedMenu == 8) tft.drawRect((bt[bt[8].ButtonNum].XButos + Xbutst + border) , (bt[bt[8].ButtonNum].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, MUTEbutcol);
  tft.setFreeFont(&Serif_bold_15);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[8].ButtonNam), ( bt[bt[8].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[8].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 2)));
  tft.drawString(MUTEbuttext, ( bt[bt[8].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[8].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 17)));
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawAGC()  {
//=======================================================================================
  int AGCbutcol;
  int isSelected;
  int border;
  si4735.getAutomaticGainControl();
  if (selectedMenu == 4) {
    isSelected = TFT_RED;
    border = 3;
  } else {
    isSelected = TFT_DARKGREY;
    border = 1;
  }    
  if (si4735.isAgcEnabled()) {
    AGCbutcol = TFT_RED;
    AGCbuttext = "ON";
  } else {
    AGCbutcol = TFT_BLUE;
    AGCbuttext = "OFF";
  }
  tft.fillRect(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, isSelected);
  tft.fillRect((bt[bt[4].ButtonNum].XButos + Xbutst + border) , (bt[bt[4].ButtonNum].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, AGCbutcol);
  if (selectedMenu == 4) tft.drawRect((bt[bt[4].ButtonNum].XButos + Xbutst + border) , (bt[bt[4].ButtonNum].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, AGCbutcol);
  tft.setFreeFont(&Serif_bold_15);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[4].ButtonNam), ( bt[bt[4].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[4].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 2)));
  tft.drawString(AGCbuttext, ( bt[bt[4].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[4].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 17)));
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawBFO()  {
//=======================================================================================
  int BFObutcol;
  int isSelected;
  int border;
  if (selectedMenu == 3) {
    isSelected = TFT_RED;
    border = 3;
  } else {
    isSelected = TFT_DARKGREY;
    border = 1;
  }    
  if (bfoOn) {
    BFObutcol = TFT_RED;
    BFObuttext = "ON";
  } else {
    BFObutcol = TFT_BLUE;
    BFObuttext = "OFF";
  }    
  tft.fillRect(bt[bt[3].ButtonNum].XButos + Xbutst, bt[bt[3].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, isSelected);
  tft.fillRect((bt[bt[3].ButtonNum].XButos + Xbutst + border) , (bt[bt[3].ButtonNum].YButos + Ybutst + border), Xbutsiz - 2*border , (Ybutsiz - 2*border), BFObutcol);
  if (selectedMenu == 3) tft.drawRect((bt[bt[3].ButtonNum].XButos + Xbutst + border) , (bt[bt[3].ButtonNum].YButos + Ybutst + border), (Xbutsiz - 2*border) , Ybutsiz - 2*border, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, BFObutcol);
  tft.setFreeFont(&Serif_bold_15);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[3].ButtonNam), ( bt[bt[3].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[3].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 2)));
  tft.drawString(BFObuttext, ( bt[bt[3].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[3].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 17)));
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawKeyPath() {
//=======================================================================================
  int Sbutcol;
  tft.fillScreen(TFT_BLACK);
  for (int n = 0 ; n <= lastKPath; n++) {
    tft.fillRect(kp[n].Xkeypos + kp[n].Xkeypnr , kp[n].Ykeypos + kp[n].Ykeypnr , kp[n].Xkeypsr , kp[n].Ykeypsr, TFT_GREEN);
    if ( n == 11) { // S button is red
      Sbutcol = TFT_RED;
    } else {
      Sbutcol = TFT_BLUE;
    }
    tft.fillRect((kp[n].Xkeypos + kp[n].Xkeypnr + 3) , (kp[n].Ykeypos + kp[n].Ykeypnr + 3), (kp[n].Xkeypsr - 5) , (kp[n].Ykeypsr - 5), Sbutcol);
    tft.setTextColor(TFT_CYAN, Sbutcol );
    tft.setFreeFont(&Serif_bold_20);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    #ifdef IhaveTDisplayTFT
      tft.drawString((Keypathtext[kp[n].KeypNum]), ( kp[n].Xkeypos + kp[n].Xkeypnr + 50), (kp[n].Ykeypos + kp[n].Ykeypnr  + 37));
    #endif
    tft.setFreeFont(NULL);
  }
}

//=======================================================================================
void HamBandlist() {
//=======================================================================================  

  for (int n = selectedHam - 2 ; n <= selectedHam + 2; n++) { // Draw only 5 options. No more is needed. Only 3 options are visible at display
    if ( n >= 0 and n <= lastHam ) { // Do not draw an inexistent option (and avoid a catastrofic expection, of course)
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedHam ) {  // Highlight the selected option
        if ( selectedHam == 0 or selectedHam == lastHam -1 ) {
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr) , (bn[n].Ybandsr), TFT_RED);    
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 3) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr + 3), (bn[n].Xbandsr - 6) , (bn[n].Ybandsr - 6), TFT_BLUE);
          tft.fillRect((bn[n+1].Xbandos + bn[n+1].Xbandnr) + Xfband, (bn[n+1].Ybandos) + (bn[n+1].Ybandnr), (bn[n+1].Xbandsr) , (bn[n+1].Ybandsr), TFT_DARKGREY);    
          tft.fillRect((bn[n+1].Xbandos + bn[n+1].Xbandnr + 1) + Xfband, (bn[n+1].Ybandos) + (bn[n+1].Ybandnr + 1), (bn[n+1].Xbandsr - 2) , (bn[n+1].Ybandsr - 2), TFT_BLUE);
          tft.drawString(band[bn[n+1].BandNum].bandName, (bn[n+1].Xbandos + bn[n+1].Xbandnr) + Xfband + Xfsize/2, (bn[n+1].Ybandos) + (bn[n+1].Ybandnr) + Yfsize/2+9);
        }
        if (selectedHam == 1 or selectedHam == lastHam ) {
          tft.fillRect((bn[n-1].Xbandos + bn[n-1].Xbandnr) + Xfband, (bn[n-1].Ybandos) + (bn[n-1].Ybandnr), (bn[n-1].Xbandsr) , (bn[n-1].Ybandsr), TFT_DARKGREY);    
          tft.fillRect((bn[n-1].Xbandos + bn[n-1].Xbandnr + 1) + Xfband, (bn[n-1].Ybandos) + (bn[n-1].Ybandnr + 1), (bn[n-1].Xbandsr - 2) , (bn[n-1].Ybandsr - 2), TFT_BLUE);
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr) , (bn[n].Ybandsr), TFT_RED);    
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 3) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr + 3), (bn[n].Xbandsr - 6) , (bn[n].Ybandsr - 6), TFT_BLUE);
          tft.drawString(band[bn[n-1].BandNum].bandName, (bn[n-1].Xbandos + bn[n-1].Xbandnr) + Xfband + Xfsize/2, (bn[n-1].Ybandos) + (bn[n-1].Ybandnr) + Yfsize/2+9);
        }
        tft.drawString(band[bn[n].BandNum].bandName, (bn[n].Xbandos + bn[n].Xbandnr) + Xfband + Xfsize/2, (bn[n].Ybandos) + (bn[n].Ybandnr) + Yfsize/2+9);
      } else {  // Draw the other options without highlight.
        tft.drawString(band[bn[n].BandNum].bandName, (bn[n].Xbandos + bn[n].Xbandnr) + Xfband + Xfsize/2, (bn[n].Ybandos) + (bn[n].Ybandnr) + Yfsize/2+9);
      }
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }

// ====================== Less code, but it has an uncomfortable flicker. Not a good experience       ==================
// ====================== Fell free to try it                                                         ==================

/*   for (int n = selectedHam - 2 ; n <= selectedHam + 2; n++) { // Draw only 5 options. No more is needed. Only 3 options are visible at display
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedHam ) {  // Highlight the selected option
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr) , (bn[n].Ybandsr), TFT_RED);    
          tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 3) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr + 3), (bn[n].Xbandsr - 6) , (bn[n].Ybandsr - 6), TFT_BLUE);
          tft.drawString(band[bn[n].BandNum].bandName, (bn[n].Xbandos + bn[n].Xbandnr) + Xfband + Xfsize/2, (bn[n].Ybandos) + (bn[n].Ybandnr) + Yfsize/2+9);
      }  else if ( n >= 0 and n <= lastHam ) {  // Draw the other options without highlight.
                tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr) , (bn[n].Ybandsr), TFT_DARKGREY);    
                tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 1) + Xfband, (bn[n].Ybandos) + (bn[n].Ybandnr + 1), (bn[n].Xbandsr - 2) , (bn[n].Ybandsr - 2), TFT_BLUE);
                tft.drawString(band[bn[n].BandNum].bandName, (bn[n].Xbandos + bn[n].Xbandnr) + Xfband + Xfsize/2, (bn[n].Ybandos) + (bn[n].Ybandnr) + Yfsize/2+9);
              }
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
  }
 */

// ====================== End - Less code, but it has an uncomfortable flicker. Not a good experience ==================

}

//=======================================================================================
void BroadBandlist() {
//=======================================================================================  

  for (int n = selectedBroad - 2 ; n <= selectedBroad + 2; n++) { // Draw only 5 options. No more is needed. Only 3 options are visible at display
    if ( n >= 0 and n <= lastBroad ) { // Do not draw an inexistent option (and avoid a catastrofic expection, of course)
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedBroad ) {  // Highlight the selected option
        if ( selectedBroad == 0 or selectedBroad == lastBroad -1 ) {
          tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr) + Xfbband, (bb[n].Ybbandos) + (bb[n].Ybbandnr), (bb[n].Xbbandsr) , (bb[n].Ybbandsr), TFT_RED);    
          tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr + 3) + Xfbband, (bb[n].Ybbandos) + (bb[n].Ybbandnr + 3), (bb[n].Xbbandsr - 6) , (bb[n].Ybbandsr - 6), TFT_BLUE);
          tft.fillRect((bb[n+1].Xbbandos + bb[n+1].Xbbandnr) + Xfbband, (bb[n+1].Ybbandos) + (bb[n+1].Ybbandnr), (bb[n+1].Xbbandsr) , (bb[n+1].Ybbandsr), TFT_DARKGREY);    
          tft.fillRect((bb[n+1].Xbbandos + bb[n+1].Xbbandnr + 1) + Xfbband, (bb[n+1].Ybbandos) + (bb[n+1].Ybbandnr + 1), (bb[n+1].Xbbandsr - 2) , (bb[n+1].Ybbandsr - 2), TFT_BLUE);
          tft.drawString(band[bb[n+1].BbandNum].bandName, (bb[n+1].Xbbandos + bb[n+1].Xbbandnr) + Xfbband + Xfbsize/2, (bb[n+1].Ybbandos) + (bb[n+1].Ybbandnr) + Yfbsize/2+9);
        }
        if (selectedBroad == 1 or selectedBroad == lastBroad ) {
          tft.fillRect((bb[n-1].Xbbandos + bb[n-1].Xbbandnr) + Xfbband, (bb[n-1].Ybbandos) + (bb[n-1].Ybbandnr), (bb[n-1].Xbbandsr) , (bb[n-1].Ybbandsr), TFT_DARKGREY);    
          tft.fillRect((bb[n-1].Xbbandos + bb[n-1].Xbbandnr + 1) + Xfbband, (bb[n-1].Ybbandos) + (bb[n-1].Ybbandnr + 1), (bb[n-1].Xbbandsr - 2) , (bb[n-1].Ybbandsr - 2), TFT_BLUE);
          tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr) + Xfbband, (bb[n].Ybbandos) + (bb[n].Ybbandnr), (bb[n].Xbbandsr) , (bb[n].Ybbandsr), TFT_RED);    
          tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr + 3) + Xfbband, (bb[n].Ybbandos) + (bb[n].Ybbandnr + 3), (bb[n].Xbbandsr - 6) , (bb[n].Ybbandsr - 6), TFT_BLUE);
          tft.drawString(band[bb[n-1].BbandNum].bandName, (bb[n-1].Xbbandos + bb[n-1].Xbbandnr) + Xfbband + Xfbsize/2, (bb[n-1].Ybbandos) + (bb[n-1].Ybbandnr) + Yfbsize/2+9);
        }
        tft.drawString(band[bb[n].BbandNum].bandName, (bb[n].Xbbandos + bb[n].Xbbandnr) + Xfbband + Xfbsize/2, (bb[n].Ybbandos) + (bb[n].Ybbandnr) + Yfbsize/2+9);
      } else {  // Draw the other options without highlight.
        tft.drawString(band[bb[n].BbandNum].bandName, (bb[n].Xbbandos + bb[n].Xbbandnr) + Xfbband + Xfbsize/2, (bb[n].Ybbandos) + (bb[n].Ybbandnr) + Yfbsize/2+9);
      }
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}

//=======================================================================================
void Steplist() {
//=======================================================================================  

  for (int n = selectedStep - 2 ; n <= selectedStep + 2; n++) {
    if ( n >= 0 and n <= lastStep ) {
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedStep ) {
        if ( selectedStep == 0 or selectedStep == lastStep -1 ) {
          tft.fillRect((sp[n].Xstepos + sp[n].Xstepnr) + Xfstep, (sp[n].Ystepos) + (sp[n].Ystepnr), (sp[n].Xstepsr) , (sp[n].Ystepsr), TFT_RED);    
          tft.fillRect((sp[n].Xstepos + sp[n].Xstepnr + 3) + Xfstep, (sp[n].Ystepos) + (sp[n].Ystepnr + 3), (sp[n].Xstepsr - 6) , (sp[n].Ystepsr - 6), TFT_BLUE);
          tft.fillRect((sp[n+1].Xstepos + sp[n+1].Xstepnr) + Xfstep, (sp[n+1].Ystepos) + (sp[n+1].Ystepnr), (sp[n+1].Xstepsr) , (sp[n+1].Ystepsr), TFT_DARKGREY);    
          tft.fillRect((sp[n+1].Xstepos + sp[n+1].Xstepnr + 1) + Xfstep, (sp[n+1].Ystepos) + (sp[n+1].Ystepnr + 1), (sp[n+1].Xstepsr - 2) , (sp[n+1].Ystepsr - 2), TFT_BLUE);
          tft.drawString(String(sp[n+1].stepFreq) + " kHz", (sp[n+1].Xstepos + sp[n+1].Xstepnr) + Xfstep + Xfbsize/2, (sp[n+1].Ystepos) + (sp[n+1].Ystepnr) + Yfbsize/2+9);
        }
        if (selectedStep == 1 or selectedStep == lastStep ) {
          tft.fillRect((sp[n-1].Xstepos + sp[n-1].Xstepnr) + Xfstep, (sp[n-1].Ystepos) + (sp[n-1].Ystepnr), (sp[n-1].Xstepsr) , (sp[n-1].Ystepsr), TFT_DARKGREY);    
          tft.fillRect((sp[n-1].Xstepos + sp[n-1].Xstepnr + 1) + Xfstep, (sp[n-1].Ystepos) + (sp[n-1].Ystepnr + 1), (sp[n-1].Xstepsr - 2) , (sp[n-1].Ystepsr - 2), TFT_BLUE);
          tft.fillRect((sp[n].Xstepos + sp[n].Xstepnr) + Xfstep, (sp[n].Ystepos) + (sp[n].Ystepnr), (sp[n].Xstepsr) , (sp[n].Ystepsr), TFT_RED);    
          tft.fillRect((sp[n].Xstepos + sp[n].Xstepnr + 3) + Xfstep, (sp[n].Ystepos) + (sp[n].Ystepnr + 3), (sp[n].Xstepsr - 6) , (sp[n].Ystepsr - 6), TFT_BLUE);
          tft.drawString(String(sp[n-1].stepFreq) + " kHz", (sp[n-1].Xstepos + sp[n-1].Xstepnr) + Xfstep + Xfbsize/2, (sp[n-1].Ystepos) + (sp[n-1].Ystepnr) + Yfbsize/2+9);
        }
        tft.drawString(String(sp[n].stepFreq) + " kHz", (sp[n].Xstepos + sp[n].Xstepnr) + Xfstep + Xfbsize/2, (sp[n].Ystepos) + (sp[n].Ystepnr) + Yfbsize/2+9);
      } else {
        tft.drawString(String(sp[n].stepFreq) + " kHz", (sp[n].Xstepos + sp[n].Xstepnr) + Xfstep + Xfbsize/2, (sp[n].Ystepos) + (sp[n].Ystepnr) + Yfbsize/2+9);
      }        
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}

//=======================================================================================
void Modelist() {
//=======================================================================================  
  for (int n = selectedMode - 2 ; n <= selectedMode + 2; n++) {
    if ( n >= 0 and n <= lastMod ) {
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedMode ) {
        if ( selectedMode == 0 or selectedMode == lastMod -1 ) {
          tft.fillRect((md[n].Xmodos + md[n].Xmodnr) + Xfmod, (md[n].Ymodos) + (md[n].Ymodnr), (md[n].Xmodsr) , (md[n].Ymodsr), TFT_RED);    
          tft.fillRect((md[n].Xmodos + md[n].Xmodnr + 3) + Xfmod, (md[n].Ymodos) + (md[n].Ymodnr + 3), (md[n].Xmodsr - 6) , (md[n].Ymodsr - 6), TFT_BLUE);
          tft.fillRect((md[n+1].Xmodos + md[n+1].Xmodnr) + Xfmod, (md[n+1].Ymodos) + (md[n+1].Ymodnr), (md[n+1].Xmodsr) , (md[n+1].Ymodsr), TFT_DARKGREY);    
          tft.fillRect((md[n+1].Xmodos + md[n+1].Xmodnr + 1) + Xfmod, (md[n+1].Ymodos) + (md[n+1].Ymodnr + 1), (md[n+1].Xmodsr - 2) , (md[n+1].Ymodsr - 2), TFT_BLUE);
          tft.drawString(bandModeDesc[md[n+1].Modenum], (md[n+1].Xmodos + md[n+1].Xmodnr) + Xfmod + Xfmodsize/2, (md[n+1].Ymodos) + (md[n+1].Ymodnr) + Yfmodsize/2+9);
        }
        if (selectedMode == 1 or selectedMode == lastMod ) {
          tft.fillRect((md[n-1].Xmodos + md[n-1].Xmodnr) + Xfmod, (md[n-1].Ymodos) + (md[n-1].Ymodnr), (md[n-1].Xmodsr) , (md[n-1].Ymodsr), TFT_DARKGREY);    
          tft.fillRect((md[n-1].Xmodos + md[n-1].Xmodnr + 1) + Xfmod, (md[n-1].Ymodos) + (md[n-1].Ymodnr + 1), (md[n-1].Xmodsr - 2) , (md[n-1].Ymodsr - 2), TFT_BLUE);
          tft.fillRect((md[n].Xmodos + md[n].Xmodnr) + Xfmod, (md[n].Ymodos) + (md[n].Ymodnr), (md[n].Xmodsr) , (md[n].Ymodsr), TFT_RED);    
          tft.fillRect((md[n].Xmodos + md[n].Xmodnr + 3) + Xfmod, (md[n].Ymodos) + (md[n].Ymodnr + 3), (md[n].Xmodsr - 6) , (md[n].Ymodsr - 6), TFT_BLUE);
          tft.drawString(bandModeDesc[md[n-1].Modenum], (md[n-1].Xmodos + md[n-1].Xmodnr) + Xfmod + Xfmodsize/2, (md[n-1].Ymodos) + (md[n-1].Ymodnr) + Yfmodsize/2+9);
        }
        tft.drawString(bandModeDesc[md[n].Modenum], (md[n].Xmodos + md[n].Xmodnr) + Xfmod + Xfmodsize/2, (md[n].Ymodos) + (md[n].Ymodnr) + Yfmodsize/2+9);
      } else {
        tft.drawString(bandModeDesc[md[n].Modenum], (md[n].Xmodos + md[n].Xmodnr) + Xfmod + Xfmodsize/2, (md[n].Ymodos) + (md[n].Ymodnr) + Yfmodsize/2+9);
      }
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}


//=======================================================================================
void BWList() {
//=======================================================================================  
  for (int n = selectedBW - 2 ; n <= selectedBW + 2; n++) {
    if ( n >= 0 and n <= nrbox ) {
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedBW ) {
        if ( selectedBW == 0 or selectedBW == nrbox -1 ) {
          tft.fillRect((bw[n].Xos + bw[n].Xnr) + XfBW, (bw[n].Yos) + (bw[n].Ynr), (bw[n].Xsr) , (bw[n].Ysr), TFT_RED);    
          tft.fillRect((bw[n].Xos + bw[n].Xnr + 3) + XfBW, (bw[n].Yos) + (bw[n].Ynr + 3), (bw[n].Xsr - 6) , (bw[n].Ysr - 6), TFT_BLUE);
          tft.fillRect((bw[n+1].Xos + bw[n+1].Xnr) + XfBW, (bw[n+1].Yos) + (bw[n+1].Ynr), (bw[n+1].Xsr) , (bw[n+1].Ysr), TFT_DARKGREY);    
          tft.fillRect((bw[n+1].Xos + bw[n+1].Xnr + 1) + XfBW, (bw[n+1].Yos) + (bw[n+1].Ynr + 1), (bw[n+1].Xsr - 2) , (bw[n+1].Ysr - 2), TFT_BLUE);
          if ( currentMode == AM) tft.drawString(bandwidthAM[bw[n+1].BandWidthAM], (bw[n+1].Xos + bw[n+1].Xnr) + XfBW + XfBWsize/2, (bw[n+1].Yos) + (bw[n+1].Ynr) + Yfmodsize/2+9);
          else tft.drawString(bandwidthSSB[bw[n+1].BandWidthSSB], (bw[n+1].Xos + bw[n+1].Xnr) + XfBW + XfBWsize/2, (bw[n+1].Yos) + (bw[n+1].Ynr) + Yfmodsize/2+9);
        }
        if (selectedBW == 1 or selectedBW == nrbox ) {
          tft.fillRect((bw[n-1].Xos + bw[n-1].Xnr) + XfBW, (bw[n-1].Yos) + (bw[n-1].Ynr), (bw[n-1].Xsr) , (bw[n-1].Ysr), TFT_DARKGREY);    
          tft.fillRect((bw[n-1].Xos + bw[n-1].Xnr + 1) + XfBW, (bw[n-1].Yos) + (bw[n-1].Ynr + 1), (bw[n-1].Xsr - 2) , (bw[n-1].Ysr - 2), TFT_BLUE);
          tft.fillRect((bw[n].Xos + bw[n].Xnr) + XfBW, (bw[n].Yos) + (bw[n].Ynr), (bw[n].Xsr) , (bw[n].Ysr), TFT_RED);    
          tft.fillRect((bw[n].Xos + bw[n].Xnr + 3) + XfBW, (bw[n].Yos) + (bw[n].Ynr + 3), (bw[n].Xsr - 6) , (bw[n].Ysr - 6), TFT_BLUE);
          if ( currentMode == AM) tft.drawString(bandwidthAM[bw[n-1].BandWidthAM], (bw[n-1].Xos + bw[n-1].Xnr) + XfBW + XfBWsize/2, (bw[n-1].Yos) + (bw[n-1].Ynr) + Yfmodsize/2+9);
          else tft.drawString(bandwidthSSB[bw[n-1].BandWidthSSB], (bw[n-1].Xos + bw[n-1].Xnr) + XfBW + XfBWsize/2, (bw[n-1].Yos) + (bw[n-1].Ynr) + Yfmodsize/2+9);
        }
        if ( currentMode == AM) tft.drawString(bandwidthAM[bw[n].BandWidthAM], (bw[n].Xos + bw[n].Xnr) + XfBW + XfBWsize/2, (bw[n].Yos) + (bw[n].Ynr) + Yfmodsize/2+9);
        else tft.drawString(bandwidthSSB[bw[n].BandWidthSSB], (bw[n].Xos + bw[n].Xnr) + XfBW + XfBWsize/2, (bw[n].Yos) + (bw[n].Ynr) + Yfmodsize/2+9);
      } else {
        if ( currentMode == AM) tft.drawString(bandwidthAM[bw[n].BandWidthAM], (bw[n].Xos + bw[n].Xnr) + XfBW + XfBWsize/2, (bw[n].Yos) + (bw[n].Ynr) + Yfmodsize/2+9);
        else tft.drawString(bandwidthSSB[bw[n].BandWidthSSB], (bw[n].Xos + bw[n].Xnr) + XfBW + XfBWsize/2, (bw[n].Yos) + (bw[n].Ynr) + Yfmodsize/2+9);
      }        
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}

//=======================================================================================
void Presetlist() {
//=======================================================================================  
  for (int n = selectedPreset - 2*0 ; n <= selectedPreset + 2*0; n++) {
    if ( n >= 0 and n <= lastPreset ) {
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BL_DATUM);
      tft.setTextPadding(tft.textWidth("88888M88888M88888M"));
      tft.drawString(String(selectedPreset + 1) + ") " + String(((preset[selectedPreset].presetIdx) / 100), 1) + " - "+ preset[n].PresetName, ((preset[n].Xpresetos + preset[n].Xpresetnr) + Xfpreset + Xfpresetsize/2)*0 + 6, (preset[n].Ypresetos) + (preset[n].Ypresetnr) + Yfpresetsize/2+9);
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}


//=======================================================================================
void Brightlist() {
//=======================================================================================  
  for (int n = selectedBright - 2 ; n <= selectedBright + 2; n++) {
    if ( n >= 0 and n <= lastBright ) {
      tft.setTextColor(TFT_CYAN, TFT_BLUE);
      tft.setFreeFont(&Serif_bold_15);
      tft.setTextSize(1);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(tft.textWidth("88888M"));
      if (n == selectedBright ) {
        if ( selectedBright == 0 or selectedBright == lastBright -1 ) {
          tft.fillRect((brt[n].XGenButos + brt[n].XGenButnr) + XfGenBut, (brt[n].YGenButos) + (brt[n].YGenButnr), (brt[n].XGenButsr) , (brt[n].YGenButsr), TFT_RED);    
          tft.fillRect((brt[n].XGenButos + brt[n].XGenButnr + 3) + XfGenBut, (brt[n].YGenButos) + (brt[n].YGenButnr + 3), (brt[n].XGenButsr - 6) , (brt[n].YGenButsr - 6), TFT_BLUE);
          tft.fillRect((brt[n+1].XGenButos + brt[n+1].XGenButnr) + XfGenBut, (brt[n+1].YGenButos) + (brt[n+1].YGenButnr), (brt[n+1].XGenButsr) , (brt[n+1].YGenButsr), TFT_DARKGREY);    
          tft.fillRect((brt[n+1].XGenButos + brt[n+1].XGenButnr + 1) + XfGenBut, (brt[n+1].YGenButos) + (brt[n+1].YGenButnr + 1), (brt[n+1].XGenButsr - 2) , (brt[n+1].YGenButsr - 2), TFT_BLUE);
          tft.drawString(Brightness[brt[n+1].GenButNum].BrightName, (brt[n+1].XGenButos + brt[n+1].XGenButnr) + XfGenBut + XfGenButsize/2, (brt[n+1].YGenButos) + (brt[n+1].YGenButnr) + YfGenButsize/2+9);
        }
        if (selectedBright == 1 or selectedBright == lastBright ) {
          tft.fillRect((brt[n-1].XGenButos + brt[n-1].XGenButnr) + XfGenBut, (brt[n-1].YGenButos) + (brt[n-1].YGenButnr), (brt[n-1].XGenButsr) , (brt[n-1].YGenButsr), TFT_DARKGREY);    
          tft.fillRect((brt[n-1].XGenButos + brt[n-1].XGenButnr + 1) + XfGenBut, (brt[n-1].YGenButos) + (brt[n-1].YGenButnr + 1), (brt[n-1].XGenButsr - 2) , (brt[n-1].YGenButsr - 2), TFT_BLUE);
          tft.fillRect((brt[n].XGenButos + brt[n].XGenButnr) + XfGenBut, (brt[n].YGenButos) + (brt[n].YGenButnr), (brt[n].XGenButsr) , (brt[n].YGenButsr), TFT_RED);    
          tft.fillRect((brt[n].XGenButos + brt[n].XGenButnr + 3) + XfGenBut, (brt[n].YGenButos) + (brt[n].YGenButnr + 3), (brt[n].XGenButsr - 6) , (brt[n].YGenButsr - 6), TFT_BLUE);
          tft.drawString(Brightness[brt[n-1].GenButNum].BrightName, (brt[n-1].XGenButos + brt[n-1].XGenButnr) + XfGenBut + XfGenButsize/2, (brt[n-1].YGenButos) + (brt[n-1].YGenButnr) + YfGenButsize/2+9);
        }
        tft.drawString(Brightness[brt[n].GenButNum].BrightName, (brt[n].XGenButos + brt[n].XGenButnr) + XfGenBut + XfGenButsize/2, (brt[n].YGenButos) + (brt[n].YGenButnr) + YfGenButsize/2+9);
      } else {
        tft.drawString(Brightness[brt[n].GenButNum].BrightName, (brt[n].XGenButos + brt[n].XGenButnr) + XfGenBut + XfGenButsize/2, (brt[n].YGenButos) + (brt[n].YGenButnr) + YfGenButsize/2+9);
      }
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
    }    
  }
}


//=======================================================================================
void subrstatus() {
//=======================================================================================
  tft.fillScreen(TFT_BLACK);
  currentFrequency = si4735.getFrequency();
  while ( 1 ) {
    tft.setFreeFont(NULL);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.println("Mod. : " + String(bandModeDesc[band[bandIdx].prefmod]) + " ");
    if ( currentMode != FM)  tft.println("Freq.    : " + String(currentFrequency, 0) + " kHz");
    else tft.println("Freq. : " + String(currentFrequency / 100, 2) + " MHz");
    si4735.getCurrentReceivedSignalQuality();
    tft.print("RSSI/SNR : " + String(si4735.getCurrentRSSI()) + "dBuV / "); // si4735.getCurrentSNR()
    tft.print(String(si4735.getCurrentSNR()) + "uV");
    if (  currentMode == FM ) {
      sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? " / STEREO    " : "  /  MONO    ");
      tft.println(String(buffer));
    } else {
      tft.println("");
    }
    si4735.getAutomaticGainControl();
    si4735.getCurrentReceivedSignalQuality();
    tft.println("LNA GAIN : " + String(si4735.getAgcGainIndex()) + "/" + String(currentAGCAtt));
    tft.println("Volume : " + String(si4735.getVolume()));
    sprintf(buffer, "%s", (si4735.isAgcEnabled()) ? "AGC ON " : "AGC OFF");
    tft.println(buffer);
    if (bfoOn) tft.println("BFO ON  ");
    else tft.println("BFO OFF ");
    tft.println("AVC max GAIN : " + String(si4735.getCurrentAvcAmMaxGain()));
    tft.println("Ant. Cap = " + String(si4735.getAntennaTuningCapacitor()));
    tft.println("BANDA : " + String(bandIdx) + "  " + String(band[bandIdx].bandName));
    tft.println("FILTRO SSB : " + String(bandwidthSSB[bwIdxSSB]) + " kHz");
    tft.println("FILTRO AM : " + String(bandwidthAM[bwIdxAM]) + " kHz");
    tft.println("PASSO : " + String(currentStep));
    //tft.drawString("Power Supply : " + String(((1.66 / 1850)*vsupply) * 2) + " V.", 120, 50);
    tft.println("SOFTWARE V1 ALPHA - NEW MENU INTERFACE");
    tft.println("BY RICARDO - GERT - THIAGO - RXAVIER");
    tft.setFreeFont(NULL);
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      delay(400);
      return;
    }
    yield();
  }
}

//=======================================================================================
void showRDSStation() {
//=======================================================================================
  tft.setCursor(XFreqDispl + 130, YFreqDispl + 62);
  tft.print(stationName);  
  delay(250);
}

//=======================================================================================
void checkRDS() {
//=======================================================================================
  si4735.getRdsStatus();
  if (si4735.getRdsReceived()) {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound() ) {
      stationName = si4735.getRdsText0A();
      tft.setTextSize(1);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextDatum(BC_DATUM);
      if ( stationName != NULL)   showRDSStation();
    }
  }
}

/*
    Prevents blinking during the frequency display.
    Erases the old digits if it has changed and print the new digit values.
*/
void showContentWithoutBlink(int col, int line, char *oldValue, const char *newValue, uint32_t color, uint8_t space, uint8_t textSize)
{

  int c = col;

  char *pOld;
  char *pNew;

  pOld = oldValue;
  pNew = (char *) newValue;
  
  // prints just changed digits
  while (*pOld && *pNew)
  {
    if (*pOld != *pNew)
    {
      tft.drawChar(c, line, *pOld, TFT_BLACK, TFT_BLACK, textSize);
      tft.drawChar(c, line, *pNew, color, color, textSize);
    }

    pOld++;
    pNew++;
    c += space;
  }

  // Is there anything else to erase?
  while (*pOld)
  {
    tft.drawChar(c, line, *pOld, TFT_BLACK, TFT_BLACK, textSize);
    pOld++;
    c += space;
  }

  // Is there anything else to print?
  while (*pNew)
  {
    tft.drawChar(c, line, *pNew, color, color, textSize);
    pNew++;
    c += space;
  }
  strcpy(oldValue, newValue);
}


char *kHz =  "kHz";
char *mhz =  "MHz";

char * formatFrequency(char *strFreq) {

  char tmp[15];
  char *unt;

  sprintf(tmp, "%5.3u", si4735.getFrequency());  // TODO sprintf(tmp, "%5.3u", si4735.getFrequency());

  strFreq[0] = tmp[0];  
  strFreq[1] = tmp[1];
  if (si4735.isCurrentTuneFM())
  {
    strFreq[2] = tmp[2];
    strFreq[3] = tmp[3];
    strFreq[4] = '\0';
    unt = mhz;
  } else {
      strFreq[2] = tmp[2];
      strFreq[3] = tmp[3];
      strFreq[4] = tmp[4];
      strFreq[5] = '\0';
      unt = kHz;
  }

  return unt;;
}

void * formatAGC(char *strAGC) {

  char tmp[15];

  sprintf(tmp, "%2.2u", currentAGCgain);  // TODO sprintf(tmp, "%5.3u", si4735.getFrequency());

  strAGC[0] = tmp[0];  
  strAGC[1] = tmp[1];
  strAGC[2] = '\0';

}

inline void showContent(uint16_t col, uint16_t lin, char *oldContent, char *newContent, const GFXfont *font, uint16_t color, uint8_t space) {
  tft.setFreeFont(font);
  showContentWithoutBlink(col, lin, oldContent, newContent, color, space, 1);
  tft.setFreeFont(NULL);
}


void cleanDispl() {
tft.fillRect( XFreqDispl + 6, YFreqDispl + 22 , 228, 45, TFT_BLACK); // Black freq. field
 cleanBuffer();
} 



void showFrequency() {
  char tmpFrequency[10];
  char tmpVFO[10];
  char *untFreq; 
  if (bfoOn) {  
     sprintf(tmpVFO,"%5d",currentBFO);
     showContent(XFreqDispl + 95, YFreqDispl + 55, bufferVFO, tmpVFO, &DSEG7_Classic_Mini_Bold_20, TFT_ORANGE, 18);
     showContent(XFreqDispl + 192, YFreqDispl + 40, bufferAux, "Hz", &Serif_bold_10, TFT_GREEN, 10);
     showContent(XFreqDispl + 192, YFreqDispl + 55, bufferAux, "BFO", &Serif_bold_15, TFT_RED, 12);
     untFreq = formatFrequency(tmpFrequency);
     showContent(XFreqDispl + 5, YFreqDispl + 55, bufferFrequency, tmpFrequency, &DSEG7_Classic_Mini_Bold_20, TFT_CYAN, 18);
  } else {    
     untFreq = formatFrequency(tmpFrequency);
     showContent(XFreqDispl + 60, YFreqDispl + 55, bufferFrequency, tmpFrequency, &DSEG7_Classic_Mini_Bold_30, TFT_CYAN, 26);
     if (band[bandIdx].bandType == FM_BAND_TYPE)
     tft.setFreeFont(&DSEG7_Classic_Mini_Bold_30);
     tft.drawChar(XFreqDispl + 60 + 78, YFreqDispl + 55, (si4735.isCurrentTuneFM())? '.':' ', TFT_CYAN, TFT_BLACK, 1);
     tft.setFreeFont(NULL);
     showContent(XFreqDispl + 192, YFreqDispl + 50, bufferUnit, untFreq, &Serif_bold_15, TFT_GREEN, 14);
     showContent(XFreqDispl + 10, YFreqDispl + 50, bufferBandName, (char *)band[bandIdx].bandName, &Serif_bold_15, TFT_RED, 12);
  }

}

void FreqDispl() {

  char tmpAux[10];

  AGCfreqdisp();
  BFOfreqdisp();
  bufferAux[0] = '\0';
  // bufferAgcGain[0] = '\0';
  if (AGCgainbut) {
/*       sprintf(tmpAux,"%2.2d",currentAGCgain);
      showContent(XFreqDispl + 20, YFreqDispl + 50, bufferAux, tmpAux, &DSEG7_Classic_Mini_Bold_20, TFT_CYAN, 20);
      showContent(XFreqDispl + 80, YFreqDispl + 50, bufferAux, "ATT SET", &Serif_bold_20, TFT_GREEN, 20);
 */
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.setFreeFont(&Serif_bold_20);
      tft.setTextSize(1);
      tft.setTextDatum(BL_DATUM);
      tft.setTextPadding(tft.textWidth("8888888888"));
      tft.drawString(String(currentAGCgain) + " ATT SET",XFreqDispl + 20, YFreqDispl + 60);
      tft.setTextDatum(BC_DATUM);
      tft.setTextPadding(0);
      tft.setFreeFont(NULL);
  } else {
    showFrequency();
  }

}


/**
 * Checks the stop seeking criterias.  
 * Returns true if the user rotates the encoder. 
 */
bool checkStopSeeking() {
  // Checks the touch
  return (bool) encoderCount;   // returns true if the user rotates the encoder
} 


void SeekFreq (uint16_t freq) {
  currentFrequency = freq;
  showFrequency();
}
   
//=======================================================================================
void DrawDispl() {
//=======================================================================================

  tft.fillRect(XFreqDispl, YFreqDispl, 240, 90, TFT_DARKGREY);
  tft.fillRect(XFreqDispl + 2, YFreqDispl + 2, 236, 86, TFT_BLACK);  
  FreqDispl();
 
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setFreeFont(&Serif_plain_10);
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString(bandModeDesc[currentMode], XFreqDispl + 81, YFreqDispl + 16);
    tft.setTextPadding(tft.textWidth("2.2kHz"));
    if (currentMode == AM) BWtext = bandwidthAM[bwIdxAM];
    else BWtext = bandwidthSSB[bwIdxSSB];
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextPadding(tft.textWidth("FT"));
    tft.drawString("FT", XFreqDispl + 112, YFreqDispl + 17);
    tft.setTextPadding(tft.textWidth("2.2kHz"));
    tft.drawString(BWtext + "kHz", XFreqDispl + 144, YFreqDispl + 17);
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ST", XFreqDispl + 183, YFreqDispl + 16);    
    tft.drawString(String(band[bandIdx].currentStep) + "kHz", XFreqDispl + 214, YFreqDispl + 16);
    tft.setFreeFont(NULL);
  }
}

//=======================================================================================
void AGCfreqdisp() {
//=======================================================================================
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setFreeFont(&Serif_plain_10);
  tft.setTextSize(1);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(tft.textWidth("AGC"));
  tft.drawString("AGC", XFreqDispl + 50, YFreqDispl + 16);
  si4735.getAutomaticGainControl();
  if (si4735.isAgcEnabled()) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString("ON", XFreqDispl + 50, YFreqDispl + 26);    
  } else {
    if (AGCgain == 0)   {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("OFF", XFreqDispl + 50, YFreqDispl + 26);
    } else {
      tft.drawString(String(currentAGCgain), XFreqDispl + 50, YFreqDispl + 26); 
      tft.setFreeFont(NULL); 
    }
  }
  tft.setTextPadding(0);
  tft.setFreeFont(NULL); 
} 


//=======================================================================================
void BFOfreqdisp() {
//=======================================================================================
if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setFreeFont(&Serif_plain_10);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(tft.textWidth("XXX"));
    tft.drawString("BFO", XFreqDispl + 20, YFreqDispl + 16);
    tft.setTextPadding(tft.textWidth("88"));  
    if (bfoOn) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString(String(currentBFOStep), XFreqDispl + 20, YFreqDispl + 26);
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.drawString("  ", XFreqDispl + 20, YFreqDispl + 26);
      tft.setFreeFont(NULL);
    }
  }
}   

//=======================================================================================
void STEPfreqdisp() {
//=======================================================================================
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setFreeFont(&Serif_plain_10);
    tft.setTextSize(1);
    tft.setTextPadding(tft.textWidth("88kHz"));
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ST", XFreqDispl + 183, YFreqDispl + 16);    
    tft.drawString(String(band[bandIdx].currentStep) + "kHz", XFreqDispl + 214, YFreqDispl + 16);
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
  }
}

