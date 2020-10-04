/*  RSL_VNA7 Arduino sketch for audio VNA measurements.
    Copyright (c) 2016-2019 Robert Larkin  W7PUA

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
e
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* RSL_VNA5.ino    v0.7.0
   Orig v0.1.0   18 Dec 2016
   Output a sine wave from the Left CODEC Line Output
   Receive that signal with Left Codec input.

   Use Right CODEC input to measure the voltage source.
   Use two mixers (multipliers) plus LPF to create i and q
   Measure sqrt(i^2 + q^2) and atan2(q, i), print to serial stream
   v0.2.0 revised to move I and Q through queue's.  RSL
   v0.2.2 brought out reference signal from Right channel. RSL
   v0.3.0 Create separate measurement function and measure power off I & Q  RSL
   v0.3.1 Removed post multiplier filters and made 1/freq a sub-multiple of
        measurement time.  Added tracking of source phase.   RSl 3 Jan 2017
   v0.4.0 test bed, added SerialCommandR
   v0.4.1 Re-did freq control structrue to recompute many variables when freq is changed.
        added setUpNewFreq() and modifyFreq()  RSL 25 Jan 2017
   v0.5.1 Revised to use Steber method of impedance determination (series R-Z). RSL 31 Jan 2017
   v0.5.2 Redid serial command structure to be orderly.RSL 3 March 17
   v0.5.3 Added corrections for input circuitry of op-amps  RSL 13 Mar 2017
   v0.5.4 Redid command structure.  Added numerous commands.  RSL 16 Mar 2017
   v0.5.6 Added ILI9341 320x240 display and touch screen.
   v0.5.7 Added vavnaState and lcdMenu[][].  Fixed LCD scan functions.  RSL 1 May 2017
   v0.5.7 Added column headings for un-annotated sweep output.  Removed baud control.
          RSL 8 Sept 2017
   v0.5.8 Added SCREEN_ROTATION to allow screen mounting up or down. Be sure XPT2046_Touchscreen.h  & .cpp
          are after Dec 2017.  RSL 23 May 18.
   v0.5.9 Added LINLOG command and LinLogCommand()  RSL 29 May 2018
   v0.6.0 Added AVNA1Config.h to hold ROTATION values, fixed serial out when under lcd control.  1 June 2018
   v0.6.1 Rewrote TUNEUP command toincrease averaging and improve accuracy.
   v0.7.0 Added sweep flexability, modularity. Added lower-case cmd's for nanoVNA-saver. Added buffering of
          incoming serial data. Added EEPROM save of Cl data and interpolation.  RSL 12 Feb 2020
   v0.7.1 Added ch> to cal cmd;
   v0.8.0 Added Vector Voltmeter, Spectrum Analyzer and 4x Sig Generators.  Added calibration for in/out levels
          to support these.  Added screen display of spectrum.  Added digit-bydigit up/down widget.
          Updated EEPROM for these. Fixed double precision constants being read as floats.
*/

// Keep version (0,255).  Rev 0.70: Not put into EEPROM byte 0 anymore.
#define CURRENT_VERSION 80

// Teensy serial/uart 4, using Teensy Pin 31 and Pin 32.
#define HWSERIAL4 Serial4
// Compile this feature or not:
#define HWSERIAL4_USED 1

#if HWSERIAL4_USED
    uint8_t incomingByte;
#endif

/* TEENSY NOTE - This was tested using a Teensy 3.6.  It probably is OK for the slower 3.5.
   Versions like 3.2 may not have sufficient speed. If a Teensy board is to be
   purchased, buy the 3.6!
   This program was tested with Arduino 1.8.2 and Teensyduino 1.36.
*/

/* The audio board uses the following pins.
    9 - BCLK
   11 - MCLK
   13 - RX
   15 - VOL    (analog pot)
   18 - SDA
   19 - SCL
   22 - TX
   23 - LRCLK
   31 - Serial 4 for 9600 baud control, RX4 in to AVNA rev 7.0
   32 - Serial 4 for 9600 baud control, TX4 out of AVNA rev 7.0
   Five pins are used for control of the AVNA:
   35 - R5K relay
   36 - 50 Ohm relay
   37 - Measuretransmission sw
   38 - Measure Impedance
   39 - Connect inputs for calibration
   Two pins control the front panel bi-color LED
   0  - Bicolor
   1  - Bicolor
   One pin inputs the weighted sum of all power supplies
   21 - Power Supply monitor
   Five pins control the 320 x 240 display:
   7  - SPI MOSI
   8  - SPI MISO
   10 - ILI9341 Display D/C
   14 - SPI SCK
   20 - Chip Select for 9341 Display
   PIns 7, 8 and 14 are also used for the same function, for touch screen
   In addition touch screen uses
   6  - Touch CS
   24 - Touch IRQ
*/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SD_t3.h>
#include <SerialFlash.h>
#include <EEPROM.h>

#include <XPT2046_Touchscreen.h>
#include "complexR.h"
#include "SerialCommandR.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include "analyze_fft1024_p.h"
 // swap() collision between ILI9341_t3 and vector.  A problem?  Order as shown stops
#include <ILI9341_t3.h>
// <font_Arial.h> from ILI9341_t3
#include <font_Arial.h>
#include "AVNA8defaultParameters.h"
// use fifo buffer for serial input of commands:
#include "CircularBuffer.h"

#define DIAGNOSTICS 0
// static params want to come from EEPROM.  Set following to 1 to reset these to initial values
#define RE_INIT_EEPROM 0

// SCREEN_ROTATION 1 is for wires on right, 3 is for wires on left, as viewed from top. 2 & 4 don't fit.
// #define SCREEN_ROTATION 1
// TOUCH_ROTATION is whatever makes the touch line up with the display.  Either 1 or 3.
// #define TOUCH_ROTATION 1
// To allow changes to SCREEN_ROTATION and TOUCH_ROTATION, they have been moved to AVNA1Config.h  rev0.60
#include "AVNA1Config.h"

// Who is in charge
#define CTRL_LCD   1
#define CTRL_USB   2

// For control of panel LED
#define LOFF 0
#define LSTART 1
#define LGREEN 2
#define LRED 3
// and the digital pins
#define PANEL_LED0 0
#define PANEL_LED1 1

// Digital hardware control pins
#define R50_36 36
#define R5K_35 35
#define CAL_39 39
#define IMPEDANCE_38 38
#define TRANSMISSION_37 37

#define FILT_NONE 0
#define LPF2300  1
#define NUM_VNAF 14

// Defines for Instrument, these are mutually exclusive
#define ASA_IDLE -2
#define VVM_IDLE -1
#define AVNA 0
#define VVM  1
#define ASA  2
#define VIN_CAL 3
#define VOUT_CAL 4
#define TOUCH_CAL 5

// For setting sample rates:
#define S6K    0
#define S12K   1
#define S24K   2
#define S44100 3
#define S44117 4
#define S48K   5
#define S96K   6
#define S100K  7
#define S192K  8
#define FBASE  44117.65f

// msec delay before measurement
#define ZDELAY 10

#define NOCAL 0
#define OPEN 1
#define THRU 2

#define IDLE     0
#define ZSINGLE  1
#define TSINGLE  2
#define ZSWEEP   3
#define TSWEEP   4
#define WHATSIT  5
#define LCDHELP  6

// defines for Serial control
#define NO_MEASURE   0
#define IMPEDANCE    1
#define TRANSMISSION 2

#define SWEEP 0
#define SINGLE 1

#define R_OFF 0
#define R50   1
#define R5K   2

// Values for doRun
#define RUNNOT     0
#define CONTINUOUS 1
#define COUNTDOWN  2
#define SINGLE_NC  3
#define POWER_SWEEP 4

// These are single precision, now
#define r2df(r) 57.29578f*r
#define d2rf(d) 0.0174532925f*d

// Nominal tft values
// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 320
#define TS_MINY 320
#define TS_MAXX 3770
#define TS_MAXY 3770

// Pin assignment for AVNA rev 0.55 board
#define TFT_DC 10
#define TFT_CS 20
#define TFT_RST 255
#define TFT_MOSI 7
#define TFT_SCLK 14
#define TFT_MISO 8

// Touch CS
#define T_CS_PIN  6
// Touch IRQ pin
#define TIRQ_PIN 24

// These are the same as Teensy Audio synthesize waveform
#define WAVEFORM_SINE              0
#define WAVEFORM_SAWTOOTH          1
#define WAVEFORM_SQUARE            2
#define WAVEFORM_TRIANGLE          3
#define WAVEFORM_ARBITRARY         4
#define WAVEFORM_PULSE             5
#define WAVEFORM_SAWTOOTH_REVERSE  6
#define WAVEFORM_SAMPLE_HOLD       7
#define WAVEFORM_TRIANGLE_VARIABLE 8

void tNothing(void);       // Declarations are required to reference function
void tToInstrumentHome(void); //   by pointer, just like C
void tToAVNAHome(void);
void tToAVNA(void);
void tToVVM(void);
void tToASA(void);
void tToASAFreq(void);
void tToASAAmplitude(void);
void tToASGHome(void);
void tToASGn(void);
void tDoSingleFreq(void);
void tDoSweep(void);
void tDoSweepAmplitude(void);
void tDoSweepZ(void);
void tDoSweepT(void);
void tSweepFreqDown(void);
void tSweepFreqUp(void);
void tWhatIntro(void);
void tDoWhatPart(void);
void tUseLF(void);
void tNoLF(void);
void tDoSettings(void);
void tDoHelp(void);
void tDoHelp2(void);
void tDoHelp5(void);
void tFreqUp(void);
void tFreqDown(void);
void tFreq0(void);
void tSet50 (void);
void tSet5K(void);
void tDoSingleZ(void);
void tDoSingleT(void);
void tCalCommand(void);
void tService(void);
void tTouchCal(void);
void tVInCal(void);
void tVOutCal(void);

// 2D array of function pointers to be called when a menu item is touched
// 15 menus and 6 buttons per menu
void (*t[22][6])(void) =
{ tToInstrumentHome, tToAVNA, tToVVM, tToASA, tToASGHome, tService,             // 0 INSTRUMENT HOME
  tToInstrumentHome, tDoSingleFreq, tDoSweep, tWhatIntro, tDoSettings, tDoHelp, // 1 AVNA Home
  tToAVNAHome, tFreqDown, tFreqUp, tFreq0, tDoSingleZ, tDoSingleT,              // 2 Single Freq
  tToAVNAHome, tSweepFreqDown, tSweepFreqUp, tNothing, tDoSweepZ, tDoSweepT,    // 3 Sweep Freq
  tToAVNAHome, tSet50,  tSet5K, tToAVNAHome, tToAVNAHome, tToAVNAHome,          // 4 Settings
  tToAVNAHome, tToAVNAHome, tToAVNAHome, tToAVNAHome, tToAVNAHome, tToAVNAHome, // 5 More Settings
  tToAVNAHome, tDoHelp, tDoHelp2, tNothing, tNothing, tNothing,                 // 6 Help
  tToAVNAHome, tDoHelp, tDoHelp2, tNothing, tNothing, tNothing,                 // 7 More Help
  tToAVNAHome, tNothing, tNothing, tNoLF, tUseLF, tDoWhatPart,                  // 8 What Component
  tToAVNAHome, tNothing, tNothing, tNothing, tCalCommand, tDoSingleT,           // 9 Single T
  tToAVNAHome, tSweepFreqDown, tSweepFreqUp, tNothing, tCalCommand, tDoSweepT,  // 10 Sweep T
  tToAVNAHome, tSweepFreqDown, tSweepFreqUp, tNothing, tCalCommand, tDoSweepZ,  // 11 Sweep Z
  tToInstrumentHome,  tToASAFreq, tToASAAmplitude, /*tToASASettings*/ tNothing, tToASGHome, tDoHelp, // 12 ASA
  tToASA, tToASAFreq, tToASAFreq, tNothing, tNothing, tNothing,                 // 13 ASA Freq
  tToASA, tToASAAmplitude, tToASAAmplitude, tToASAAmplitude, tToASAAmplitude, tDoHelp,  // 14 ASA Amplitude
  tToInstrumentHome, tToASGn, tToASGn, tToASGn, tToASGn, tNothing,              // 15 ASG Home
  tToASGHome, tToASGn, tToASGn, tToASGn, tToASGn, tNothing,                     // 16 ASG N 
  tToInstrumentHome, tToVVM, tToVVM, tToVVM, tNothing, tNothing,                // 17 VVM Home
  tToInstrumentHome, tTouchCal, tVInCal, tVOutCal, tNothing, tNothing,          // 18 Service
  tNothing, tNothing, tNothing, tNothing, tNothing, tNothing,                   // 19 Touch Cal
  tService, tVInCal, tNothing, tNothing, tNothing, tService,                    // 20 VIn Cal
  tService, tVOutCal, tNothing, tNothing, tNothing, tService                    // 21 VOut Cal
};

//=============================  OBJECTS  ========================================

SerialCommand SCmd;   // The SerialCommand object to control AVNA
//       SerialCommand SCmd;   // The SerialCommand object to control AVNA  <<<<<<<<<ADD for Port 4<<<
// 320 x 240 Screen object
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

// XPT2046_Touchscreen ts(T_CS_PIN);  // Param 2 - NULL - No interrupts
// XPT2046_Touchscreen ts(T_CS_PIN, 255);  // Param 2 - 255 - No interrupts
XPT2046_Touchscreen ts(T_CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

//Audio objects          instance name
//Added for 4-channel sig gen
AudioSynthWaveform       sgWaveform[4];
AudioMixer4              mixer1;          // Combine 4 waveforms
AudioMixer4              mixer2;          // Add waveforms to AVNA source
AudioConnection          patchCordp(sgWaveform[0], 0, mixer1, 0);
AudioConnection          patchCordq(sgWaveform[1], 0, mixer1, 1);
AudioConnection          patchCordr(sgWaveform[2], 0, mixer1, 2);
AudioConnection          patchCords(sgWaveform[3], 0, mixer1, 3);
AudioConnection          patchCordt(mixer1,      0, mixer2, 0);
// Regular AVNA objects
AudioControlSGTL5000     audioShield;
AudioInputI2S            audioInput;      // Measurement signal
AudioAnalyzeFFT1024_p    fft1024p;
AudioFilterFIR           firIn1;
AudioFilterFIR           firIn2;
AudioSynthWaveform       waveform1;       // Test signal
AudioSynthWaveform       waveform2;       // 90 degree
AudioEffectMultiply      mult1;           // I mixer
AudioEffectMultiply      mult2;           // Q mixer
AudioSynthWaveformDc     DC1;             // Gain control for DAC output
AudioEffectMultiply      multGainDAC;
AudioOutputI2S           i2s2;
AudioRecordQueue         queueNN[4];      // Two pairs of I and Q
// And for the reference channel
AudioEffectMultiply      multR1;          // I mixer
AudioEffectMultiply      multR2;          // Q mixer
// Additional measurments
AudioAnalyzePeak         pkDet;           // These 2 watch for overload
AudioAnalyzePeak         pkDetR;
AudioAnalyzeRMS          rms1;
//                                  Into cord     Out-of cord
//                                 coming from    going to
AudioConnection          patchCord4(audioInput, 0, fft1024p, 0);    // Transmission ADC to FFTAudioAnalyzeFFT1024_p
AudioConnection          patchCord0(waveform1, 0, multGainDAC, 0);  // Test signal
AudioConnection          patchCord0A(DC1, 0,      multGainDAC, 1);  // Set gain
//Was AudioConnection          patchCord0C(multGainDAC, 0, i2s2, 0);    // Test signal to L output
AudioConnection          patchCord0C(multGainDAC, 0, mixer2, 1);    // Test signal to L output
AudioConnection          patchCord1C(mixer2, 0, i2s2, 0);           // Test signal to L output
// I-Q multipliers
AudioConnection          patchCord1(waveform1, 0, mult1, 1);        // In phase LO
AudioConnection          patchCord2(waveform2, 0, mult2, 1);        // Out of phase LO
AudioConnection          patchCordF1(audioInput, 0, firIn1, 0);     // Left input to FIR LPF
AudioConnection          patchCordF2(firIn1, 0, mult1, 0);          // FIR LPF to mixer I
AudioConnection          patchCordF3(firIn1, 0, mult2, 0);          // FIR LPF to mixer Q
AudioConnection          patchCord5(mult1, 0,   queueNN[0], 0);     // Q
AudioConnection          patchCord6(mult2, 0,   queueNN[1], 0);     // I
AudioConnection          patchCordpp(audioInput, 0, pkDet, 0);      // 2.0 max p-p
// For ref channel
AudioConnection          patchCordR1(waveform1, 0, multR1, 1);      // In phase LO
AudioConnection          patchCordR2(waveform2, 0, multR2, 1);      // Out of phase LO
AudioConnection          patchCordG1(audioInput, 1, firIn2, 0);     // Left input to FIR LPF
AudioConnection          patchCordG2(firIn2, 0, multR1, 0);         // FIR LPF to mixer I
AudioConnection          patchCordG3(firIn2, 0, multR2, 0);         // FIR LPF to mixer Q
AudioConnection          patchCordR5(multR1, 0, queueNN[2], 0);     // RQ
AudioConnection          patchCordR6(multR2, 0, queueNN[3], 0);     // RI
AudioConnection          patchCordRp(audioInput, 1, pkDetR, 0);
AudioConnection          pc100(audioInput, 0, rms1, 0);


// "Complex" objects are a set of two ordered float32
// Object array as a vectors, Z, Y and T
std::vector<Complex> Z;
std::vector<Complex> Y;
std::vector<Complex> T;

Complex Cone(1.0f, 0.0f);          // Complex number 1.0 + j0.0

CircularBuffer<char, 1000> serInBuffer;
boolean commandOpen = true;

// ================================  Global Variables  =====================================
// Expanding te AVNA to other instruments adds the need to change behavior depending on
// AVNA, VVM or ASA.  To do this:
int16_t instrument = AVNA;

// LPF, 100 taps, 2300 Hz Fco, 100 KHz sample rate
// -20 dB stopband
static int16_t lp2300_100K[100] = {
  1083, -751, -514, -336, -201, -99, -20, 42, 90, 130,
  163, 188, 205, 213, 215, 207, 188, 159, 124, 75,
  21, -40, -105, -173, -239, -300, -353, -394, -420, -428,
  -415, -379, -318, -232, -121, 13, 171, 346, 538, 741,
  950, 1160, 1366, 1561, 1740, 1897, 2029, 2132, 2203, 2238,
  2238, 2203, 2132, 2029, 1897, 1740, 1561, 1366, 1160, 950,
  741, 538, 346, 171, 13, -121, -232, -318, -379, -415,
  -428, -420, -394, -353, -300, -239, -173, -105, -40, 21,
  75, 124, 159, 188, 207, 215, 213, 205, 188, 163,
  130, 90, 42, -20, -99, -201, -336, -514, -751, 1083
};

/*  Frequency Table
    {0.0, 0.0} are complex zeros.
    Frequencies are channelized for AVNA4, with the exception
    that FreqData0] is not part of the sweep and can be used as a
    manually controlled frequency. Thirteen frequencies are
    used for the sweep from 10 to 40,000 Hz.  42 bytes/freq
*/
#define AMP_PHASE ((double)1.0L), ((double)0.0L)
struct measureFreq {
  float    freqHz;            // Requested frequency (display freq, also)
  float    freqHzActual;      // Achieved frequency (used in line 2 data)
  uint16_t numTenths;
  double   vRatio;            // Vor/Vom magnitudes 
  double   dPhase;            // phaseor - phaseom
  double   thruRefAmpl;       // Keep these 4 as double for historic (EEPROM) reasons
  double   thruRefPhase;
} FreqData[NUM_VNAF] =
{ 1000.0f,   1000.0f,  5, AMP_PHASE,  AMP_PHASE,   //if freq<911 int(22.4-3.14*ln(freq) else 1
  10.0f,       10.0f, 20, AMP_PHASE,  AMP_PHASE,
  20.0f,       20.0f, 10, AMP_PHASE,  AMP_PHASE,
  50.0f,       50.0f, 10, AMP_PHASE,  AMP_PHASE,
  100.0f,     100.0f,  5, AMP_PHASE,  AMP_PHASE,
  200.0f,     200.0f,  5, AMP_PHASE,  AMP_PHASE,
  500.0f,     500.0f,  2, AMP_PHASE,  AMP_PHASE,
  1000.0f,   1000.0f,  1, AMP_PHASE,  AMP_PHASE,
  2000.0f,   2000.0f,  1, AMP_PHASE,  AMP_PHASE,
  5000.0f,   5000.0f,  1, AMP_PHASE,  AMP_PHASE,
  10000.0f, 10000.0f,  1, AMP_PHASE,  AMP_PHASE,
  20000.0f, 20000.0f,  1, AMP_PHASE,  AMP_PHASE,
  30000.0f, 30001.0f,  1, AMP_PHASE,  AMP_PHASE,
  40000.0f, 39998.0f,  1, AMP_PHASE,  AMP_PHASE
};

// Some data items to support the AVNA looking like a nanoVNA to a
//nanoVNA-saver program on a serial link.
uint16_t sweepStart = 100;
uint16_t sweepStop  = 40000;
uint16_t sweepPoints = 101;
// sweepCurrentPoint controls whether data is collected. Setting to
// zero starts things and arriving at sweepPoints stops it.
uint16_t sweepCurrentPoint = 101;

// sendDataType  0=sending S11 data
//               1=sending S21 data
uint16_t sendDataType = 0;
// sendDataCount is next data point to send. -1=not sending.
int16_t sendDataCount = -1;
// Need to keep a "total" in case another quantity is set by sweep command
uint16_t totalDataPoints = 0;

//  ===  Variables added to support nanoVNA-saver   ===
// Not measuring, notsending data
#define NO_NANO  0
// Measuring S11 and S21
#define MEASURE_NANO 1
// Measuring is done and data is ready to send
#define DATA_READY_NANO 2
// Need to keep track of whether doing regular serial with
// cap commands or Nano support with l.c. commands
boolean doingNano = false;
uint16_t nanoState = NO_NANO;

// Data arrays S11 and S21:
float dataFreq[1601];
float dataReReflec[1601];
float dataImReflec[1601];
float dataReTrans[1601];
float dataImTrans[1601];

// portSelect                |------ Use USB Serial for nanoVNA-saver data
//                           ||------Use HWSERIAL4 for  nanoVNA-saver data
uint8_t portSelect = 0B00000011;
#define NANO_USE_HW4 0B00000001
#define NANO_USE_USB 0B00000010
//              ===  End of nanoVNA variables  ===

uint16_t nFreq = 0;             // 1.0 kHz default
uint16_t nFreqDisplay = 3;      // For LCD sweep display; start, plus 6 higher
uint16_t num256blocks;          // These control sampling time
uint16_t numCycles;
/* In getting data from audio objects, we have need for
    four different data types.  These 'NN' arrays are
    0 - Q or out-of-phase measurement
    1 - I or in-phase measurement
    2 - RQ or out-of-phase measurement of reference channel
    3 - RI or in-phase measurement of reference channel
*/
boolean   doTuneup = false;  // Change print during tuneup
boolean   NNReady[4] = {false, false, false, false};
float32_t sumNN[4];
double    superAveNN[4];
int16_t   bufferNN[4][256];

struct sigGen {  // Structure defined here.  See EEPROM save for asg[]
    float32_t freq;
    float32_t amplitude;
    bool ASGoutputOn;
    short type;
} asg[4] = {
    1030.0f,  0.2828f, false, WAVEFORM_SINE,
    1400.0f,  0.05f,   false, WAVEFORM_SINE,
    5000.0f,   0.0f,   false, WAVEFORM_SINE,
    200.0f,    0.0f,   false, WAVEFORM_SQUARE};
 

//          ----------------------------------------------------------
// Data to be saved when updated and used at power up.
struct saveState {
  uint16_t startup;        // Detect valid EEPROM
  float    freqHz0;        // Frequency for nFreq = 0
  float    freqHzActual0;  // This also
  uint16_t msDelay;
  uint16_t ZorT;
  uint16_t SingleorSweep;
  uint16_t iRefR;
  float    valueRRef[3];
  float    vMult;
  // capInput and resInput are used to compensate for high frequency, high impedance
  // measurements.  These are the stray capacity of the Measure wiring, and R1 1 Megohm.
  float    capInput;
  float    resInput;
  // capCouple compensates for the 0.22 uF at the R or M amp, and effects low frequencies.
  float    capCouple;
  // seriesR and seriesL compensate for the leads going to the DUT, and effect high frequencies
  // and low impedance measurements.
  float    seriesR;
  float    seriesL;
  // Next four set the data formats.  See LinLogCommand()  rev 0.59
  uint8_t  rsData;
  uint8_t  tsData;
  uint8_t  rdData;
  uint8_t  tdData;
  // Save calibration for user point + 13 quasi-log points (added rev 0.70):
  double   EvRatio[14];            // Vor/Vom magnitudes
  double   EdPhase[14];            // phaseor - phaseom
  double   EthruRefAmpl[14];
  double   EthruRefPhase[14];
  uint8_t  int8version;
  // The following were added at rev 0.80 Sept 2020.  Adds 26 bytes to 546 total
  // TFT corner values, settable from Touch Sceen Cal.
  uint16_t xMin;    // TS_MINX; 320 Default
  uint16_t yMin;    // TS_MINY; 320
  uint16_t xMax;    // TS_MAXX; 3770
  uint16_t yMax;    // TS_MAXY; 3770
  float32_t sgCal;  // 1.5792f;
  float32_t VVMCalConstant;    // 0.0000137798f;
  float32_t SAcalCorrectionDB; // 4.5f;
  }; 

// Set up a union of bytes that allow EEPROM storage of VNA state when turned off. Init goes with
// first member
union {
  saveState lastState;   // laststate instance of the big saveState structure, just above.
  uint8_t save_vnaF[sizeof(saveState)];  // Single byte equivalent for EEPROM storage/retrieval
} uSave = DEFAULT_PARAMETERS;
// The values above are the "default values."  The variables will be changed by data from EEPROM to get
// the user selected values.  These defaults will never appear unless commanded by PARAM1 command,
// or by the RE_INIT_DEFAULT define.  DEFAULT_PARAMETERS is in AVNA7defaultParameters.h
//           -----------------------------------------------------------

bool     calZSingle = false;
bool     calZSweep = false;
bool     calTSingle = false;
bool     calTSweep = false;
bool     dataValidTSweep = false;
bool     dataValidZSweep = false;
uint16_t doRun = RUNNOT;  //  Set of measurements or sweeps, also CONTINUOUS or COUNTDOWN
uint16_t nRun = 0;        // count down until 0
// Control of the AVNA comes from either the USB communications, or from the touch screen,
// at start up. The touch screen can be turned off by command.  The USB command is always running..
// Output is to either USB or to the screen display, as controlled by useUSB and useLCD.
bool useUSB = true;
bool useLCD = true;

// There are difference between USB commands and LCD menu operations. The LCD is more automatic
// and the USB is more detailed.  Power up is with the LCD and the primary control can be changed
// via the USB CONTROL command.
uint16_t primaryControl = CTRL_LCD;
uint16_t avnaState = 0;
// Next two allow seeing what menu call has been used
uint16_t currentMenu = 0;
uint16_t menuItem;

// The table of menu numbers sets the next menu to be called, base on the
// current menu, and the menu position touched.
uint16_t lcdMenu[22][6]    // Select next as 0 to 6. Indices are [current menu][menu item]
  = {0, 1, 17, 12, 15, 18,   // Menu 0 Instrument HOME
     0, 2, 3, 8, 4, 6,       // Menu 1 AVNA Home
     1, 2, 2, 2, 2, 9,       // Menu 2 Single Frequency
     1, 3, 3, 3, 11, 10,     // Menu 3 Sweep Frequency
     1, 4, 4, 4, 4, 4,       // Menu 4 Settings
     1, 1, 1, 1, 1, 1,       // Menu 5 More settings
     1, 6, 7, 6, 6, 6,       // Menu 6 Help
     1, 6, 7, 7, 7, 7,       // Menu 7 More help
     1, 8, 8, 8, 8, 8,       // Menu 8 What Component
     1, 9, 9, 9, 9, 9,       // Menu 9 Measure single transmission
     1, 10, 10, 10, 10, 10,  // Menu 10 Measure swept transmission
     1, 11, 11, 11, 11, 11,  // Menu 11 Measure swept impedance
     0, 13, 14, 12, 12, 12,  // Menu 12 Spectrum analyzer Home
    12, 13, 13, 13, 13, 13,  // Menu 13 Spectrum analyzer frequency
    12, 14, 14, 14, 14, 14,  // Menu 14 Spectrum analyzer amplitude
     0, 16, 16, 16, 16, 15,  // Menu 15 Signal Generator Home
    15, 16, 16, 16, 16, 16,  // Menu 16 Signal Generator N
     0, 17, 17, 17, 17, 17,  // Menu 17 Vector Voltmeter
     0, 19, 20, 21, 18, 18,  // Menu 18 Service
    18, 19, 19, 19, 19, 19,  // Menu 19 Touch Cal 
    18, 20, 20, 20, 20, 18,  // Menu 20 V In Cal 
    18, 21, 21, 21, 21, 18   // Menu 21 V Out Cal 
    };

uint16_t nSampleRate = S100K;     // Current rate,  1 is S44117 or 44711.65Hz rate
float32_t sampleRateExact = 1.000000E8;   // = 100000000.00;
float32_t factorFreq = 0.4411764706f;

Complex Ztuneup(0.0f, 0.0f);   // Adjusted for the 0.22 uF coupling cap
Complex Ztuneup0(0.0f, 0.0f);  // No 0.22 uF adjustment
Complex Tmeas(0.0f, 0.0f);          // Result of measureT()
// Control serial printing for Z measurement (both can be true):
bool seriesRX = true;
bool parallelRX = true;
// annotate refers to serial print of CSV or annotated with "Freq=", etc.
bool annotate = true;
// verboseData serial prints the inner-workings.  Not normally done.
bool verboseData = false;
bool parallelGB = false;     // LCD, parallel-model single-freq, display G-B or parallel R and X
uint16_t firstLCDSweepFreq = 3;

float dacSweep = 0.0f;   // Level during sweep
float dacLevel = 0.4f;

// VNAMode 0-Single Freq, 0 to 9, 1-Sweep at nFreq, 2-Sweep with delay
uint16_t VNAMode = 1;
// Final results
double amplitudeV, amplitudeR;
double phaseV, phaseR;

float sLC[15] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float pLC[15] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float Q[15]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float RetLoss, ReflPhase, pwrf;
Complex ReflCoeff(0.0f, 0.0f);
Complex pwr(0.0f, 0.0f);    // Complex since computed as V*Vconj

boolean printReady = false;
uint32_t countMeasurements;   // The number achieved at a freq
unsigned long timeUL;
uint16_t teststate = 0;
uint16_t test_ry = 0;
uint16_t test_sw = 0;

boolean wastouched = true;  // Touch screen
boolean whatUseLF = true;   // Use 10, 20, 50 Hz for What? (speed up)
// Note - Screen corner data is  with EEPROM

// The level of waveforms is (0.0, 1.0) but the amplitude for the sig gens 
// is 0 to 1.22 Volts p-p.  sgCal converts between these. (make settable?)
//  float32_t sgCal = 1.5792f;   With EEPROM Data
int16_t currentSigGen = 0;  // Ranges (0,3)

// The vector voltmeter has an offset of displayed phase
float32_t phaseOffset = 0.0f;

// Set by Service/InputCal. This is default value. Used in VVMMeasure
// float32_t VVMCalConstant = 0.0000137798f; With EEPROM Data

#define VVM_VOLTS 1
#define VVM_DBM 2
// Data can be either high input-Z volts or 50-Ohm dBm
int16_t VVMFormat = VVM_VOLTS;

// Save the freq[0] from the AVNA when using VVM
float32_t saveFreq0;

// The units for displaying RL and C values (2 char ea). Until we have an omega, will use 'O' as Ohms:
char *rUnits[7]={(char*)"pO", (char*)"nO", (char*)"uO", (char*)"mO", (char*)"  ", (char*)" K", (char*)" M"};
// Conductance basic unit is the mho (Ohm spelled backwards). Abbreviating as m produces some unfamiliar units:
char *gUnits[7]={(char*)"pm", (char*)"nm", (char*)"um", (char*)"mm", (char*)" m", (char*)"Km", (char*)"Mm"};
// Inductor and then capacitor units
char *lUnits[7]={(char*)"pH", (char*)"nH", (char*)"uH", (char*)"mH", (char*)" H", (char*)"kH", (char*)"MF"};
char *cUnits[7]={(char*)"pF", (char*)"nF", (char*)"uF", (char*)"mF", (char*)" F", (char*)"kF", (char*)"MF"};

unsigned long tms;

// Variables added with the Spectrum Analyzer
int16_t pixelnew[256];
int16_t pixelold[256];
int16_t spectrum_mov_average = 0;
int  spectrum_y = 15;
int  spectrum_x = 45;
int  spectrum_height = 160;
float32_t dbPerDiv = 10.0f;
float32_t ASAdbOffset = 0.0f;  // 0, 5, 10 dB, etc
float32_t specMax, specMaxFreq;
// float32_t SAcalCorrectionDB = 4.5;   With EEPROM Data
//  float32_t khzPerDiv = 4.0f;
//  float32_t khzOffset = 0.0f;
uint16_t ASAI2SFreqIndex = 4;  // 96 kHz sample rate
uint16_t countMax = 100;  //  Make user adjustable
uint16_t countAve = 0;     // Set to zero to start new average
struct frequencyData {
    float32_t sampleRate;
    uint16_t rateIndex;
    float32_t maxFreq;
    char name[8];
    uint16_t SAnAve;
} freqASA[6] = {
    6000.0f, S6K, 2.5f, "6 kHz", 16,
    12000.0f, S12K, 5.0f, "12 kHz", 16,
    24000.0f, S24K, 10.0f, "24 kHz", 32,
    48000.0f, S48K, 20.0f, "48 kHz", 64,
    96000.0f, S96K, 40.0f, "96 kHz", 128,
    192000.0f, S192K, 80.0f, "192 kHz", 200};
float32_t pwr10, pwr10DB;
// ----------- End Spectrum analyzer variables -------

// Variables to support BMP Screen Saves to SD Card
const int chipSelect = BUILTIN_SDCARD; // for Teensy 3.6
bool SDCardAvailable = false;
bool bmpScreenSDCardRequest = false;
bool hexScreenRequest = false;
File bmpFile;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

//===================================================================================
// To use STL Vector container, we need to trap some errors.
// See https://forum.pjrc.com/threads/23467-Using-std-vector  #10 Thanks, davidthings!
namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory"); while(1);
  }

  void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);  while(1);
  }
}
// ====================================================================================


// ==============================  SETUP  =============================================
void setup()
  {
  // Cause big array size to show in compile summary (needs 1+ entries):
  dataFreq[0]=10;
  dataReReflec[0]=1.0;  dataImReflec[0]=0.0;  dataReTrans[0]=1.0;  dataImTrans[0]=0.0;

  uint16_t i;
  panelLED(LSTART);
  panelLED(LRED);
  Serial.begin(9600);     // 9600 is not used, it is always 12E6
  delay(1000);            //Wait for USB serial
  // X.reserve(nn) immediately sets memory for nn X-elements.
  Z.reserve(201);
  Y.reserve(201);
  T.reserve(201);
  // Create 3 x 201 Complex objects, one at a time
  for(i=0; i<201; i++)
     {
     Z.push_back(Complex(0.0, 0.0));  // Add an element to end of vector
     Y.push_back(Complex(0.0, 0.0));
     T.push_back(Complex(0.0, 0.0));
     }

  // The following #if allows a way to reinitialize the EEPROM settings by not reading them here.
  // Not normally needed.  Rev 0.70: Makes the two bytes 0X3B and 0X43, and do NOT track version.
  // but the int8version byte does.  Also, use the "version" serial command to find the firmware version.
#if !RE_INIT_EEPROM
  if((EEPROM.read(0)==0X21 || EEPROM.read(0)==0X3B) && EEPROM.read(1)==0X43)
     {
     Serial.println("Loading EEPROM data");
     loadStateEEPROM();
     }
#endif
  saveStateEEPROM();   // Initialize EEPROM, coming from init values for saveState

  // SD Card, see what is available
  exploreSDCard();
  // Check if SD card is present and mark in SDCardAvailable
  SDCardAvailable = card.init(SPI_HALF_SPEED, chipSelect);

  tft.begin();
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(ILI9341_BLACK);
  topLines();
  tft.setFont(Arial_8);
  tft.setTextColor(ILI9341_WHITE);
  writeMenus(0);
  tToInstrumentHome();             // Adds Welcome message
  ts.begin();            // Touch screen
  ts.setRotation(TOUCH_ROTATION);   // rev 0.60
  pinMode(R50_36, OUTPUT);    // Five hardware control pins
  pinMode(R5K_35, OUTPUT);
  pinMode(CAL_39, OUTPUT);
  pinMode(IMPEDANCE_38, OUTPUT);
  pinMode(TRANSMISSION_37, OUTPUT);
  for(i=0; i<3; i++)   // Clean relay contacts
      {
      setRefR(R5K);
      delay(10);
      setRefR(R50);
      delay(10);
      }
  // For the FET switch, select, at most, one of the following three HIGH:
  digitalWrite(CAL_39, HIGH);       // On
  digitalWrite(IMPEDANCE_38, LOW);  // Off
  digitalWrite(TRANSMISSION_37, LOW);  // Off
  panelLED(LGREEN);
  Serial.print("AVNA_");
  Serial.println((float)CURRENT_VERSION/100.0);
  HWSERIAL4.print("AVNA_");
  HWSERIAL4.println((float)CURRENT_VERSION/100.0);
  Serial.print("Voltage Check (expect 145 to 175): ");
  Serial.println(analogRead(21));


#if 0
  // Following deals with a quirk of the Teensy in using double data types.  Contrary to C convention,
  // numbers such as 1.2345678901, or 1.23, are treated as floats, not doubles. To avoid this, the compiler option
  // -fsingle-precision-constant is removed from the Arduino IDE Teensy 3.6 description.
  // This test sees if this has been done.  Thanks to Mike Runyon for all the info and the test.
  double dPI =3.141592653589793L;
  if(PI != dPI)
     {
     Serial.println("WARNING: '-fsingle-precision-constant' is enabled in ...hardware/teensy/avr/boards.txt");
     Serial.println("         Minor errors may occur in calculations.");
     Serial.println("         See www.janbob.com/electron/AVNA1/AVNA1.htm#DoublePrecision for info.");
     tft.setTextColor(ILI9341_RED);
     tft.setFont(Arial_9);
     tft.setCursor(0, 167);
     tft.print("      WARNING: Using Single Precision");
     tft.setCursor(0, 184);
     tft.print("      See Serial Monitor for details.");
     }
// BELIEVED TO BE SOLVED
#endif


  // Each additional AudioMemory uses 256 bytes of RAM (dynamic memory).
  // This stores 128-16-bit ints.   Dec 16 usage 8
  AudioMemory(35);   // Last seen peaking at 20
#if 0
  // Create a synthetic sine wave, for testing
  // To use this, edit the connections above
  sinewave.amplitude(0.8);
  sinewave.frequency(1034.007);
#endif
  AudioNoInterrupts();     // Use to synchronize all audio
  // Enable the SGTL5000 audio CODEC
  audioShield.enable();
  // Headphone volume, 0.0 to 0.8 is undistorted over full input ADC range
  audioShield.volume(0.0);
  /* Adjust the sensitivity/gain  of the line-level inputs, same foor both.
    Fifteen settings are possible for  lineInLevel(both), in 1.5 dB steps:
    0: 3.12 Volts p-p  0 dB gain  (pair with output 13 for net zero dB)
    1: 2.63 Volts p-p                               16
    2: 2.22 Volts p-p                               19
    3: 1.87 Volts p-p                               22
    4: 1.58 Volts p-p                               25
    5: 1.33 Volts p-p  (default)                    28
    6: 1.11 Volts p-p
    7: 0.94 Volts p-p  More gain
    8: 0.79 Volts p-p   is lower
    9: 0.67 Volts p-p     |
    10: 0.56 Volts p-p     |
    11: 0.48 Volts p-p     V
    12: 0.40 Volts p-p
    13: 0.34 Volts p-p
    14: 0.29 Volts p-p
    15: 0.24 Volts p-p
    For output, 19 sets 2.2 v p-p range.
  */
  audioShield.lineInLevel(1, 1);    // both 5 for full bridge
  audioShield.lineOutLevel(29);  // 29 lowers the output to not overdrive the amp
  // Next, see https://forum.pjrc.com/threads/27215-24-bit-audio-boards?p=78831&viewfull=1#post78831
  audioShield.adcHighPassFilterDisable();
  waveform1.begin(WAVEFORM_SINE);
  waveform1.frequency(factorFreq * FreqData[nFreq].freqHz);
  waveform1.amplitude(1.0);
  waveform1.phase(0);
  waveform2.begin(WAVEFORM_SINE);
  waveform2.frequency(factorFreq * FreqData[nFreq].freqHz);
  waveform2.amplitude(1.0);
  waveform2.phase(270);
  firIn1.begin(lp2300_100K, 100);
  firIn2.begin(lp2300_100K, 100);
  AudioInterrupts();
  DC1.amplitude(dacLevel);                          // Gain control for DAC output
  //factorFreq is global that corrects any call involving absolute frequency, like waveform generation.
  factorFreq = FBASE / sampleRateExact;    // 44117.65f / 
  // Setup callbacks for SerialCommand commands

  // Configure the ASA window algorithm to use
  fft1024p.windowFunction(AudioWindowHanning1024);

  SCmd.addCommand("ZMEAS", ZmeasCommand);
  SCmd.addCommand("Z", ZmeasCommand);
  SCmd.addCommand("TRANSMISSION", TransmissionCommand);
  SCmd.addCommand("T", TransmissionCommand);
  SCmd.addCommand("FREQ", FreqCommand);
  SCmd.addCommand("F", FreqCommand);
  SCmd.addCommand("SWEEP", SweepCommand);
  SCmd.addCommand("CAL", CalCommand);
  SCmd.addCommand("C", CalCommand);
  SCmd.addCommand("RUN", RunCommand);
  SCmd.addCommand("R", RunCommand);
  SCmd.addCommand("POWER", PowerSweepCommand);
  SCmd.addCommand("SAVE", saveStateEEPROM);
  SCmd.addCommand("LOAD", loadStateEEPROM);
  SCmd.addCommand("DELAY", DelayCommand);
  SCmd.addCommand("CALDAT", CalDatCommand);
  SCmd.addCommand("SERPAR", SerParCommand);
  SCmd.addCommand("TEST", TestCommand);
  SCmd.addCommand("PARAM1", Param1Command);    // R50, R5K values and also use defaults
  SCmd.addCommand("PARAM2", Param2Command);    // Impedance correction factors`
  SCmd.addCommand("TUNEUP", TuneupCommand);    // Estimate strays
  SCmd.addCommand("LINLOG", LinLogCommand);    // linear units or dB, 0 to 4 parameters  - Rev 0.5.9
  SCmd.addCommand("DUMP_E", DumpECommand);     // Serial Print EEPROM, 0 parameters
  SCmd.addCommand("WRITE_E", WriteECommand);   // Write a byte to EEPROM, 2 parameters
  SCmd.addCommand("BAUD", BaudCommand);        // Change serial baud rate
  SCmd.addCommand("ANNOTATE", AnnotateCommand);
  SCmd.addCommand("VERBOSE", VerboseCommand);
  SCmd.addCommand("CALSAVE", CalSaveCommand);   // Added to save cal in EEPROM. 1 param. ver .70  Jan 2020
  // A group of commands to allow the AVNA to mimic a nanoVNA to run nanoVNA-saver control program.
  // No caps in command names, fortunately for "sweep" etc.
  SCmd.addCommand("info", infoCommand);
  SCmd.addCommand("help", helpCommand);
  SCmd.addCommand("cal", calCommand);
  SCmd.addCommand("capture", captureCommand);
  SCmd.addCommand("version", versionCommand);
  SCmd.addCommand("frequencies", frequenciesCommand);
  SCmd.addCommand("data", dataCommand);
  SCmd.addCommand("sweep", sweepCommand);
  SCmd.addCommand("resume", resumeCommand);
  SCmd.addDefaultHandler(unrecognized);         // Handler for command that isn't matched
  // Response variations:  ECHO_FULL_COMMAND replies with full received line, even if command is not valid.
  // ECHO_COMMAND ((first token only) and ECHO_OK only respond if command is valid.  With EOL.
  // ECHO_ONE responds with numerical 1 if command is valid and numerical 0 if invalid.  No EOL.
  // NO_RESPONSE  does that.
  SCmd.setResponse(ECHO_FULL_COMMAND);
  VNAMode = 1;
  uSave.lastState.msDelay = 700;   // Needs adjustable param
  Serial.println("");
  tms = millis();

  // Initialize the support for a nanoVNA-like interface (no activity settings)
  doingNano = false;  // set True for info or version, false for any cap command
  sendDataCount = 1602;    // Not sending
  totalDataPoints = 101;   // If nanoVNA-saver needs data before measuring
  sweepPoints=101;
  sweepCurrentPoint = 1602; // Not measuring

  // For VVM
  saveFreq0 = FreqData[0].freqHz;
  // And some filler data
  for (i=0; i<1601; i++)
      {
      dataFreq[i] = 2000.0 + 10.0*(float)i;
      dataReReflec[i]=0.5;
      dataImReflec[i]=0.5;
      dataReTrans[i]=0.5;
      dataImTrans[i]=0.5;
      }

  // Initialize the four signal generators synth_waveform 
  // via "begin(float t_amp, float t_freq, short t_type)"
  for (unsigned int ii = 0; ii<4; ii++)
    {
    sgWaveform[ii].begin(uSave.lastState.sgCal*asg[ii].amplitude, asg[ii].freq, asg[ii].type);
    // and the on/off is set by the mixer1 via "void gain(unsigned int channel, float gain)"
    if (asg[ii].ASGoutputOn)
      mixer1.gain(ii, 1.0f);
    else
      mixer1.gain(ii, 0.0f);
    }

  // The following sets up serial port 4, 9600 baud RS-232 control of the AVNA using comands from the
  // nanoVNA.  This is compatible with using the following for control:
  //      nanoVNA-saver.py
#if HWSERIAL4_USED
  HWSERIAL4.begin(9600);
#endif
  //And clear the serial buffers
  while (Serial.available() > 0)
    Serial.read();
  while( !serInBuffer.isEmpty() )
    serInBuffer.pop();
  }

// =====================================  LOOP  ============================================
void loop()
  {
  int16_t cmdResult;
  char inChar;
  uint16_t boxNum;

  // Serial i/o can come from either the USB Serial or from UART HWSERIAL4 connected to a RS-232 serial.
  // Check if either has data and send all that is available to try to build a command. It is the operator
  // that needs to prevent data coming from both, that is not difficult.  comm
  while (Serial.available() > 0)
    {
    inChar = Serial.read();   // Read an available character, there may be more waiting
    serInBuffer.unshift(inChar);
    // SCmd.processCh(inChar);
    }
  if(commandOpen)
    {
    while( !serInBuffer.isEmpty() )
      SCmd.processCh(serInBuffer.pop());
    }

  //The HWSERIAL needs similar treatment
  while (HWSERIAL4.available() > 0)
    {
    inChar = HWSERIAL4.read();   // Same, but for UART 4
    SCmd.processCh(inChar);
    // cmdResult = SCmd.processCh(inChar);
    // Returns 0=still filling buffer,  1=invalid command found  2=valid command found
    // if(cmdResult == 1)  HWSERIAL4.println("What cmd?");
    // if(cmdResult == 2)  HWSERIAL4.println("OK");
    }
 
  if (instrument == ASA)
    doFFT();

  if(instrument == VVM)
    VVMMeasure();

  if(instrument==AVNA)
    {
    // Following if's will execute each loop() and are for touch control
    if(avnaState == 0)     // Idle
      {
      ;
      }
    else if(avnaState==ZSINGLE)
      {
      // This is for LCD control, not serial.  Always repetitive measurements until
      // stopped by a touch entry.
      setRefR(uSave.lastState.iRefR);    // Select relay for reference resistor
      setSwitch(IMPEDANCE_38);           // Connect for Z measure
      doZSingle(true);
      delay(1000);
      }
   else if(avnaState == TSINGLE)         // T Meas 1 Freq
      {
      setRefR(uSave.lastState.iRefR);    // Select relay for reference resistor
      setSwitch(TRANSMISSION_37);        // Connect for Z measure
      doTSingle(true);
      delay(1000);
      }
   else if(avnaState == ZSWEEP)
     {
     ;  // These are empty at this point.
     }
   else if(avnaState == TSWEEP)
     {
     ;
     }
   else if(avnaState == WHATSIT)      //What Component
     {
     doWhatPart();
     avnaState = IDLE;               // Force only one run
     }
   else if(avnaState == LCDHELP)
     {
     ; // No processing required
     }
   }     // End if instrument==AVNA

  // Check if SD card is present and mark in SDCardAvailable
  SDCardAvailable = card.init(SPI_HALF_SPEED, chipSelect);

  // TOUCH SCREEN
  boolean istouched = ts.touched();
  if (istouched  &&  (millis()-tms) > 150)  // Set repeat rate and de-bounce or T_REPEAT
    {
    avnaState = 0;     // Stop measurements
    doRun = RUNNOT;       // Stop measurements
#if DIAGNOSTICS
    Serial.println(millis()-tms);
    tms = millis();
#endif
    TS_Point p = ts.getPoint();
    /*
    Serial.print("X = "); Serial.print(p.x);
    Serial.print("\tY = "); Serial.print(p.y);
    Serial.print("\tPressure = "); Serial.println(p.z);
    */
    //  Upper right 50x50 is screen save button
    if(p.x>touchx(270)  &&  p.y<touchy(50)  &&  SDCardAvailable && instrument!=TOUCH_CAL)
      {
      drawScreenSaveBox(ILI9341_RED);
      bmpScreenSDCardRequest = true;
      dumpScreenToSD();
      drawScreenSaveBox(ILI9341_GREEN);
      }
    if(p.y > touchy(198))     //  3200)   // The bottom row of menu choices
       {
       menuItem = getBoxN(p.x, 6);   // Returns (0, 5)
       // Act on menu item.  t is an 2-D array of pointers to functions to do what is needed
       // based on currentMenu and menuItem
       (*t[currentMenu][menuItem]) ();

       currentMenu = lcdMenu[currentMenu][menuItem];   // Select new menu ( or leave the same)
       writeMenus(currentMenu);                // Show the new menu
       }

     if(currentMenu == 16)   // Four Sig Gen setting menus
       {
       /* Touch screen
        * Frequency Up   y = 27 to 52 pixel
        * Frequency Down y = 75 to 100 pixel
        * Level Up       y = 117 to 142 pixel
        * Level Down     y = 165 to 190 pixel
        * All: x=(60, 100)  (100,140)  (140, 180)  (180,220)  (220, 260) pixels
        */
       boxNum = getBox7(p.x);
       if (p.y >= touchy(27) && p.y < touchy(52))  // Increment freq
         {
         switch(boxNum)
           {
           case 5: if(asg[currentSigGen].freq < 23999) asg[currentSigGen].freq += 1; break;
           case 4: if(asg[currentSigGen].freq < 23989) asg[currentSigGen].freq += 10; break;
           case 3: if(asg[currentSigGen].freq < 23899) asg[currentSigGen].freq += 100; break;
           case 2: if(asg[currentSigGen].freq < 22999) asg[currentSigGen].freq += 1000; break;
           case 1: if(asg[currentSigGen].freq < 12999) asg[currentSigGen].freq += 10000; break;
           }
         tToASGn();    // includes setSigGens()
         }
       else if (p.y >= touchy(75) && p.y < touchy(100))  // Decrement frequency
         {
         switch(boxNum)
           {
           case 5: if(asg[currentSigGen].freq >= 1) asg[currentSigGen].freq -= 1; break;
           case 4: if(asg[currentSigGen].freq >= 10) asg[currentSigGen].freq -= 10; break;
           case 3: if(asg[currentSigGen].freq >= 100) asg[currentSigGen].freq -= 100; break;
           case 2: if(asg[currentSigGen].freq >= 1000) asg[currentSigGen].freq -= 1000; break;
           case 1: if(asg[currentSigGen].freq >= 10000) asg[currentSigGen].freq -= 10000; break;
           }
         tToASGn();
         }
       else if (p.y >= touchy(117) && p.y < touchy(142))  // Increment amplitude
         {
         switch(boxNum)
           {
           case 4: if(asg[currentSigGen].amplitude < 1.199f) asg[currentSigGen].amplitude += 0.001f; break;
           case 3: if(asg[currentSigGen].amplitude < 1.189f) asg[currentSigGen].amplitude += 0.01f; break;
           case 2: if(asg[currentSigGen].amplitude < 1.099f) asg[currentSigGen].amplitude += 0.1f; break;
           case 1: if(asg[currentSigGen].amplitude < 0.199f) asg[currentSigGen].amplitude += 1.0f; break;
           }
         tToASGn();
         }
       else if (p.y >= touchy(165) && p.y < touchy(190))  // Decrement amplitude
         {
         switch(boxNum)
           {
           case 4: if(asg[currentSigGen].amplitude >= 0.001f) asg[currentSigGen].amplitude -= 0.001f; break;
           case 3: if(asg[currentSigGen].amplitude >= 0.01f)  asg[currentSigGen].amplitude -= 0.01f; break;
           case 2: if(asg[currentSigGen].amplitude >= 0.1f)   asg[currentSigGen].amplitude -= 0.1f; break;
           case 1: if(asg[currentSigGen].amplitude >= 1.0f)   asg[currentSigGen].amplitude -= 1.0f; break;
           }
         tToASGn();    // includes setSigGens()
         }
       }  // End, if currentMenu==16 (SigGen N)

     else if(currentMenu == 17)   // VVM menus
       {
       /* Touch screen
        * Phase Offset Up   y = 117 to 142 pixel
        * Phase Offset Down y = 165 to 190 pixel
        * All: x=(60, 100)  (100,140)  (140, 180)  (180,220)  (220, 260) pixels
        */
       boxNum = getBox7(p.x);
       if (p.y >= touchy(117) && p.y < touchy(142))  // Increment phaseOffset
         {
         switch(boxNum)
           {
           case 4: if(phaseOffset <= 179.9f) phaseOffset += 0.1f; tToVVM(); break;
           case 3: if(phaseOffset <= 179.0f) phaseOffset += 1.0f; tToVVM(); break;
           case 2: if(phaseOffset <= 170.0f) phaseOffset += 10.0f; tToVVM(); break;
           case 1: if(phaseOffset <= 80.0f)  phaseOffset += 100.0f; tToVVM(); break;
           }
         }
       else if (p.y >= touchy(165) && p.y < touchy(190))  // Decrement phaseOffset
         {
         switch(boxNum)
           {
           case 4: if(phaseOffset >= -179.9f) phaseOffset -= 0.1f; tToVVM(); break;
           case 3: if(phaseOffset >= -179.0f) phaseOffset -= 1.0f; tToVVM(); break;
           case 2: if(phaseOffset >= -179.0f) phaseOffset -= 10.0f; tToVVM(); break;
           case 1: if(phaseOffset >= -80.0f)  phaseOffset -= 100.0f; tToVVM(); break;
           }
         }
       }  // End, if currentMenu==17 (VVM)
    wastouched = istouched;
    delay(150);  // Reduces multiple touches on an intended one
    }    // End, screen was touched

 //if( rms1.available() ) 
 //   { Serial.println(rms1.read(), 4); delay(500); }

  if(doRun == POWER_SWEEP)
     {
     dacSweep *= 1.04;
     dacLevel = dacSweep;
     if(dacLevel > 0.5)
        {
        doRun = RUNNOT;         // Pack up like we were never here
        dacLevel = 0.4;
        }
     delay(10);
     }

  // SERIAL CONTROL
  if(doRun != RUNNOT && doRun != POWER_SWEEP)
    {
    if(teststate == 1)
      ;
    else     // Not test, response for serial control
      {
      if (nRun == 0)  // Counted down, time to stop   // REARRANGE
        doRun = RUNNOT;
      else if (doRun == COUNTDOWN)
        nRun--;
      // Sort out Sweep or Single;  Impedance or Transmission
      if (doRun == COUNTDOWN || doRun == CONTINUOUS || doRun ==SINGLE_NC)
        {
        if (uSave.lastState.ZorT == IMPEDANCE)   // Measure Z
           {
           setRefR(uSave.lastState.iRefR);    // Select relay for reference resistor
           setSwitch(IMPEDANCE_38);
           if (uSave.lastState.SingleorSweep == SWEEP)
              {
              doZSweep(true, 1);     //  Do a impedance measurement with sweep
              if(useLCD)
                 LCDPrintSweepZ();
              }
          else
              {
              doZSingle(true);
              }
           }
        else                      // TRANSMISSION
           {
           setRefR(uSave.lastState.iRefR);   // Set relay for Ref R
           setSwitch(TRANSMISSION_37);
           if (uSave.lastState.SingleorSweep == SWEEP)
              doTSweep(true);     //  Do a transmission measurement with sweep
           else
              doTSingle(true);
              if(doRun==SINGLE_NC) doRun = RUNNOT;
           }
        delay(uSave.lastState.msDelay);       // Slow down, if desired

        }      // End repetitive loop
     }  // End not test
  }  // End Serial Control

#if DIAGNOSTICS
  Serial.print("Proc = ");
  Serial.print(AudioProcessorUsage());
  Serial.print(" (");
  Serial.print(AudioProcessorUsageMax());
  Serial.print("),  Mem = ");
  Serial.print(AudioMemoryUsage());
  Serial.print(" (");
  Serial.print(AudioMemoryUsageMax());
  Serial.println(")");
#endif

  // For mimicking the nanoVNA, need to gather data, a point at a time, and
  // send data, a line at a time.  See "sweep f1 f2 n" and "data k" commands
  if(doingNano && nanoState == MEASURE_NANO)
    {
    if(sweepCurrentPoint < sweepPoints)
       getDataPt(sweepCurrentPoint++);         // Both reflecton and transmission
    else if(sweepCurrentPoint == sweepPoints)   // Done collecting data
       {
       //sendEOT();  Don't want "ch> " after sweep
       sweepCurrentPoint++;   // Once is enough
       nanoState = DATA_READY_NANO;
       commandOpen = true;      // We held up commands.  Let them in
       }
    }

  // And in about the same way, send data lines
  if(doingNano && nanoState == DATA_READY_NANO)
     {
     if(sendDataCount < totalDataPoints)
        sendDataLine(sendDataCount++);
     else if(sendDataCount == totalDataPoints)
        {
        sendEOT();            // Announce the end of data 0 or 1
        sendDataCount++;      // So we don't do EOT again
        nanoState = NO_NANO;   // End of serial data output
        commandOpen = true;
        }
     }
  }
//  =================  END LOOP()  ======================


// Returns the 1 of N touch boxes that is most likely for the x-value given by px.
// N=6 for bottom menus and N=7 for up/down boxes.
uint16_t getBoxN(uint16_t px, uint16_t N)
  {
  float32_t  xPixelEst, nf;

  xPixelEst = 320.0f*(float32_t)(px - uSave.lastState.xMin)/(float32_t)(uSave.lastState.xMax - uSave.lastState.xMin);  // about 0 to 320
  if (xPixelEst < 0.0f)  xPixelEst = 0.0f;
  if(xPixelEst > 320.0f)  xPixelEst = 320.0f;
  nf = 320.0f/(float32_t)N;
   /*
   Serial.print("1 of N: "); Serial.println( (uint16_t)(0.5 + (xPixelEst - 0.5f*nf)/nf) );
   Serial.print("px  N = "); Serial.print(px); Serial.print(",  "); Serial.println(touchy(N));
   Serial.print("xPixelEst  nf = "); Serial.print(xPixelEst); Serial.print(",  "); Serial.println(nf);
   Serial.print("Return value, int = ");  Serial.println((uint16_t)(0.5 + (xPixelEst - 0.5f*nf)/nf));
    */
  return  (uint16_t)(0.5 + (xPixelEst - 0.5f*nf)/nf);  // 0 to N-1
  }

// Returns the 1 of 7 touch boxes that is most likely for the x-value given by px.
uint16_t getBox7(uint16_t px)
  {
  int16_t  xPixelEst;  // Due to errors, this can be negative
  xPixelEst = (uint16_t)(0.5 + 320.0f*(float32_t)(px - uSave.lastState.xMin)/(float32_t)(uSave.lastState.xMax - uSave.lastState.xMin));

  // Serial.print("1 of 7: "); Serial.println((uint16_t)((xPixelEst - 20)/40));
  return  (uint16_t)((xPixelEst - 20)/40);
  }

// Returns touch screen coordinate (xMin, xMax) for lcd screen pixelX
uint16_t touchx(uint16_t pixelX)
  {
  return  uSave.lastState.xMin + (((uSave.lastState.xMax - uSave.lastState.xMin) * pixelX) / 320);
  }

// Same for y
uint16_t touchy(uint16_t pixelY)
  {
  return  uSave.lastState.yMin + (((uSave.lastState.yMax - uSave.lastState.yMin) * pixelY) / 240);
  }

void drawScreenSaveBox(uint16_t lineColor)
  {
  tft.fillRect(303, 1, 16, 18, ILI9341_BLACK);
  tft.drawRect(303, 1, 16, 18, lineColor);
  if(lineColor != ILI9341_BLACK)
    {
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_8);
    tft.setCursor(308, 6);
    tft.print("S");
    }
  tft.setTextColor(ILI9341_YELLOW);
  }

void asgPrintAmplitude(void)
  {
  tft.setCursor(95, 142);
  tft.setFont(Arial_18);
  if(asg[currentSigGen].ASGoutputOn)
    {
    printDigits((uint16_t)1000.0*asg[currentSigGen].amplitude, 4, 75, 144, true, ILI9341_YELLOW);
    tft.setTextColor(ILI9341_YELLOW);
    }
  else
    {
    printDigits((uint16_t)1000.0*asg[currentSigGen].amplitude, 4, 75, 144, true, ILI9341_PINK);
    tft.setTextColor(ILI9341_PINK);
    }
  tft.print(".");
  }

// The setup and cal info is saved to a block,
// in EEPROM.  This is the byte array save_vnaF[] that
// is unioned with the saveState structure.
void saveStateEEPROM(void)
  {
  // uSave.lastState.int8version = 70;

  uint16_t i;
  for (i = 0; i < sizeof(saveState); i++)
     EEPROM.write(i, uSave.save_vnaF[i]);
  }

// Reverse to read state from EEPROM.  If the data structure changes, and has not been
// read, EEPROM.read(0) will be out of date (lower than the CURRENT_VER), and this
// will trigger an update of save_vnaF[] and a rewrite of the EEPROM.  Then read(0)
// will be up to date and no further writes of the EEPROM is needed until a change,
// such as a re calibration is performed.
void loadStateEEPROM(void)
  {
  uint16_t i, flag;

  for (i = 0; i < sizeof(saveState); i++)
     uSave.save_vnaF[i] = EEPROM.read(i);   // Byte at a time
  // Need to move a copy of the calibration data into the working area.
  //This is separate to allow cals to be done but not saved to EEPROM. This
  // can be particularly usefuf for transmission cal.
  for(i=0; i<=13; i++)
    {
    FreqData[i].vRatio = uSave.lastState.EvRatio[i];
    FreqData[i].dPhase = uSave.lastState.EdPhase[i];
    FreqData[i].thruRefAmpl = uSave.lastState.EthruRefAmpl[i];
    FreqData[i].thruRefPhase = uSave.lastState.EthruRefPhase[i];
    }
  // Rev 0.70  Removed the changing of byte 0 to follow the version.
  // Allow for old versions of EEPROM save, that may not have calibration data:
  // For ref, note:
  //     union {
  //       saveState lastState;   // laststate instance of the big saveState structure
  //       uint8_t save_vnaF[sizeof(saveState)];
  //       } uSave = DEFAULT_PARAMETERS;
  Serial.print("Turn-on EEPROM version was "); Serial.println(uSave.lastState.int8version);

  //  uSave.lastState.int8version=59;  <<<<FOR EEPROM TESTING

  // At this point, if version is out-of-date, there will be garbage bytes read from the EEPROM
  // Now update the RAM copy and then save to EEPROM to fill those bytes.
  flag = 0;  // Nothing to write yet
  if(uSave.lastState.int8version < 70)   // EEPROM data is "way" out-of-date
     {
     // Cal data save was added with version 0.70.  Default values are here
     // but should be replaced by doing a careful "CALSAVE 0 nAve" and "CALSAVE 1 nAve" using
     // control from a serial terminal, Arduino or otherwise.
     for(i=0; i<=13; i++)
       {
       // Save ideal calibration for user point + 13 quasi-log points (added rev 0.70):
       uSave.lastState.EvRatio[i] = 1.0;
       uSave.lastState.EdPhase[i] = 0.0;
       uSave.lastState.EthruRefAmpl[i] = 1.0;
       uSave.lastState.EthruRefPhase[i] = 0.0;
       }
     uSave.save_vnaF[0] = 0X3B;
     uSave.lastState.int8version = 70;     // Mark as version 0.70
     Serial.println("EEPROM data first updated to that of version 0.70");
     Serial.println("A new serial (not touch screen) CALSAVE 0 n  and  CALSAVE 1 n  are needed.");
     flag = 1;
     }

  if(uSave.lastState.int8version < 80)   // EEPROM data is out-of-date
     {
     // Cal data save was added with version 0.70.  Default values are here
     // but should be replaced by doing a careful "CALSAVE 0 nAve" and "CALSAVE 1 nAve" using
     // control from a serial terminal, Arduino or otherwise.
     uSave.lastState.xMin = TS_MINX;  // 320
     uSave.lastState.yMin = TS_MINY;  // 320
     uSave.lastState.xMax = TS_MAXX;  // 3770
     uSave.lastState.yMax = TS_MAXY;  // 3770
     uSave.lastState.sgCal = 1.5792f;
     uSave.lastState.VVMCalConstant = 0.0000137798f;
     uSave.lastState.SAcalCorrectionDB = 4.5f;
     uSave.lastState.int8version = CURRENT_VERSION;     // Mark as version 0.80
     Serial.println("EEPROM data has been updated to that of version 0.80");
     Serial.println("including default cal values for Vin and Vout.");
     flag = 1;
     }
   if(flag)
      saveStateEEPROM();               // Udpate EEPROM to current version

   // else the EEPROM was correct, and we use that data by doing nothing
   // VNA Calibration data is a special case, in that there are both default EEPROM values
   // and temporary sets in that can be used by the CAL n command.  At startup,
   // we should make them both the same:
   for (i=0; i<=13; i++)
        {
        FreqData[i].vRatio = uSave.lastState.EvRatio[i];
        FreqData[i].dPhase = uSave.lastState.EdPhase[i];
        FreqData[i].thruRefAmpl =  uSave.lastState.EthruRefAmpl[i];
        FreqData[i].thruRefPhase = uSave.lastState.EthruRefPhase[i];
        }
    Serial.print("EEPROM Load of "); Serial.print(sizeof(saveState)); Serial.println(" bytes");
  }

/* prepMeasure(float freq)  -  Does steps neede to make the data structure
 * FreqData[0] ready to do a measurement.  NOT for FreqData 1 to 13; those are direct from table.
 * nFreq = 0 must be set before this is called.
 *  1-Find the freqHzActual frequency, range 10 to 40,000 Hz
 *  2-Find the numTenths smoothing needed
 *  3-Interpolate the corrections of the analog amplifiers
 */
void prepMeasure(float freq)
  {
  uint16_t kk;

  if (freq<=10.0)
    freq = 10.0;
  else if (freq >=39998.0006)
    freq = 39998.0006;
  modifyFreq((double)freq);  // This sets .freqHzActual
  for(kk=2; kk<=13; kk++)       // Now, interpolate the corrections
     {
     if(FreqData[0].freqHzActual < FreqData[kk].freqHzActual) // 4 linear interpolations
       {
       FreqData[0].vRatio = FreqData[kk-1].vRatio +
         ((FreqData[0].freqHzActual - FreqData[kk-1].freqHzActual)/
         (FreqData[kk].freqHzActual - FreqData[kk-1].freqHzActual)) *
         (FreqData[kk].vRatio  - FreqData[kk-1].vRatio);
       // and the same for the other three...
       FreqData[0].dPhase = FreqData[kk-1].vRatio +
         ((FreqData[0].freqHzActual - FreqData[kk-1].freqHzActual)/
         (FreqData[kk].freqHzActual - FreqData[kk-1].freqHzActual)) *
         (FreqData[kk].dPhase  - FreqData[kk-1].dPhase);
       FreqData[0].thruRefAmpl = FreqData[kk-1].thruRefAmpl +
         ((FreqData[0].freqHzActual - FreqData[kk-1].freqHzActual)/
         (FreqData[kk].freqHzActual - FreqData[kk-1].freqHzActual)) *
         (FreqData[kk].thruRefAmpl  - FreqData[kk-1].thruRefAmpl);
       FreqData[0].thruRefPhase = FreqData[kk-1].thruRefPhase +
         ((FreqData[0].freqHzActual - FreqData[kk-1].freqHzActual)/
         (FreqData[kk].freqHzActual - FreqData[kk-1].freqHzActual)) *
         (FreqData[kk].thruRefPhase  - FreqData[kk-1].thruRefPhase);
       }
     }
     // Lower frequencies are slow and noisy, so increase averaging per:
     if(FreqData[0].freqHz < 600)
       FreqData[0].numTenths = int(0.5+24.1-3.5885*log(FreqData[0].freqHz));
     else
       FreqData[0].numTenths = 1;

    saveFreq0 = FreqData[0].freqHz;  // Keep current in case VVM is used
#if 0
Serial.print("Freq =");
Serial.print(FreqData[0].freqHz); Serial.print(",");
Serial.println(FreqData[0].freqHzActual, 5);
Serial.print("4x corr and num=");
Serial.print(FreqData[0].vRatio, 5); Serial.print(",");
Serial.print(FreqData[0].dPhase, 3); Serial.print(",");
Serial.print(FreqData[0].thruRefAmpl, 5); Serial.print(",");
Serial.print(FreqData[0].thruRefPhase, 3); Serial.print(",");
Serial.println(FreqData[0].numTenths);
#endif
  }

// Output a line to Serial over USB.  Annotated or not, and corresponding
// to frequency index iF.  Assumes that useUSB is in effect
void serialPrintZ(uint16_t iF)
  {
  float rcamp, rcphase;
  Complex Zo(uSave.lastState.valueRRef[uSave.lastState.iRefR], 0.0);

  ReflCoeff = (Z[iF] - Zo) / (Z[iF] + Zo);
  pwr = ReflCoeff * ReflCoeff.conjugate();
  pwrf = pwr.real();
  RetLoss = -10.0*log10f(pwrf);
  ReflPhase = r2df(ReflCoeff.phase());
  if(uSave.lastState.rsData == 0)   // Print reflection coefficient in dB (Return Loss) and phase
    {
    Serial.print(FreqData[iF].freqHz, 3);
      // Reflection Coefficient.  Annotation is optional.
    if (annotate)
         Serial.print(" Hz Return Loss = ");
    else
         Serial.print(",");
    Serial.print(RetLoss, 3);

    if (annotate)
         Serial.print(" dB  Phase = ");
    else
         Serial.print(",");
    Serial.println(r2df(ReflCoeff.phase()), 2);
    }
  else if(uSave.lastState.rsData == 1)   // Print reflection coefficient in  magnitude and phase
    {

    Serial.print(FreqData[iF].freqHz, 3);
      // Reflection Coefficient.  Annotation is optional.
    if (annotate)
         Serial.print(" Hz Reflection Coefficient = ");
    else
         Serial.print(",");
    Serial.print((float)ReflCoeff.modulus(), 5);

    if (annotate)
         Serial.print("  Phase = ");
    else
         Serial.print(",");
    Serial.println(ReflPhase, 2);

    }
  else             // uSave.lastState.tsData == 2  Print series/parallel R, X
    {
    if(seriesRX)         // Series R and X values, plus L or C needed
      {
      // Always output freq
      Serial.print(FreqData[iF].freqHz, 3);
      // Output series Z,  L or C, and Q.  Annotation is optional.
      if (annotate)
         Serial.print(" Hz Series RX: R=");
      else
         Serial.print(",");
      Serial.print(Z[iF].real(), 3);
      if (annotate)
         Serial.print(" X=");
      else
         Serial.print(",");
      Serial.print(Z[iF].imag(), 3);
      // Serial print C or L value
      if (Z[iF].imag() < 0.0)
        {
        if (annotate)
          {
          Serial.print(" C=");
          Serial.print(valueString(sLC[iF], cUnits));
          Serial.print(" Q=");
          }
        else       // just CSV
          {
          Serial.print(",");
          Serial.print(sLC[iF], 12);
          Serial.print(",");
          }
        Serial.println(Q[iF]);
        }
      else    // Looks like an inductor
        {
        if (annotate)
          {
          Serial.print(" L= ");
           Serial.print(valueString(sLC[iF], lUnits));
          Serial.print(" Q=");
          }
        else       // just CSV
          {
          Serial.print(",");
          Serial.print(sLC[iF], 9);
          Serial.print(",");
          }
        Serial.println(Q[iF]);
        }
      }
    if(parallelRX)   // Not exclusive with series RX
      {
      Serial.print(FreqData[iF].freqHz, 3);
      // Output Y, L or C, and Q.  Annotation is optional.
      if (annotate)
         Serial.print(" Hz Parallel GB: G=");
      else
         Serial.print(",");
      Serial.print(Y[iF].real(), 9);
      if (annotate)
         Serial.print(" B=");
      else
         Serial.print(",");
      Serial.print(Y[iF].imag(), 9);
      if (annotate)
         Serial.print(" R= ");
      else
         Serial.print(",");
      Serial.print(1.0/Y[iF].real());
      if (Y[iF].imag() >= 0.0)
        {
        if (annotate)
          {
          Serial.print(" C=");
          Serial.print(valueString(pLC[iF], cUnits));
          Serial.print(" Q=");
          }
        else       // just CSV
          {
          Serial.print(",");
          Serial.print(pLC[iF], 12);
          Serial.print(",");
          }
        Serial.println(Q[iF]);
        }
      else    // Looks like an inductor
        {
        if (annotate)
          {
          Serial.print(" L= ");
          Serial.print(valueString(sLC[iF], lUnits));
          Serial.print(" Q=");
          }
        else       // just CSV
          {
          Serial.print(",");
          Serial.print(sLC[iF], 9);
          Serial.print(",");
          }
        Serial.println(Q[iF]);
        }
      }
    }      // End if(uSave.lastState.tsData == 2)  R & X
  }        // end serialPrintZ(uint16_t iF)

// -------------------------------------------------------------------------
/*  Not having filtering after the mixers improves the transient
    performance greatly.  But, it requires that one or more cycles
    of the measuring frequency fit exactly into the averaging period.
    Part of this is done by selecting the number of measurements to
    be averaged.  The frequency is then offset from the "desired" very
    slightly to make the fit "exact." The init below is exact frequency,
    the number of samples taken, the number of full 256 block samples
    used, the number of samples less than 256 needed to fill out the
    number of samples, the number of 0.1 sec measurements for AVE=1,
    the analog phase offset and finally the "almost" frequency to display.
*/
/* Request frequency f.  Depending on the sample rate, a slightly different
   frequency will be used, and set.  That frequency is returned here.
   Call this after the Sample Rate has been set (nSampleRate on 0,4).
*/
float32_t modifyFreq(float32_t f)
  {
  // Converted to all float32   ver .80
  float32_t tMeasMin, dNmin, dN, nSamples, time_samples, time_freq, fNew;
  uint16_t nSamplesI;

  tMeasMin = 0.1f;             // At least 0.1 sec
  // Figure number of f periods, including part periods
  dNmin = tMeasMin * f;
  if ((dNmin - floorf(dNmin)) < 0.000001f)
    dNmin = dNmin - 1;
  // dN is number of whole input frequency cycles
  dN = floorf(dNmin) + 1;
  nSamples = floorf(0.5f + (sampleRateExact * dN / f));   // Whole number of ADC samples
  nSamplesI = (uint16_t)nSamples;

  // Leave nSamples as is and alter freq to fit
  time_samples = nSamples / sampleRateExact;     // Time for ADC full sampling
  time_freq = dN / f;        // Time for N cycles of f

  // Alter f for 'perfect' fit.  This is the frequency from the VNA
  fNew = f * time_freq / time_samples;

  // Number of 256 word blocks and remainder.   Global variables
  num256blocks = (uint16_t) (nSamplesI / 256);
  numCycles = (uint16_t)(nSamplesI - 256 * num256blocks);
  FreqData[nFreq].freqHzActual = fNew;
#if DIAGNOSTICS
  Serial.print("Freq desired="); Serial.print(f); Serial.print(" Sample Rate="); Serial.print(sampleRateExact);
         Serial.print(" Num periods="); Serial.println(dN);
  Serial.print("Num adc samp="); Serial.print(nSamplesI); Serial.print(" Altered f="); Serial.print(fNew, 5);
#endif
  return fNew;
  }

void setSample(uint16_t nS)
  {
  sampleRateExact = (float32_t)setI2SFreq(nS);  // It returns a double
  //factorFreq is global that corrects any call involving absolute frequency, like waveform generation.
  factorFreq = FBASE / sampleRateExact;
#if DIAGNOSTICS
  Serial.print(" S Rate=");  Serial.print(sampleRateExact); Serial.print( "ff=");
  Serial.println(factorFreq);
#endif
  // The 4 sig gens need to be reprogrammed for new factorFreq
  for (unsigned int ii = 0; ii<4; ii++)
    sgWaveform[ii].begin(uSave.lastState.sgCal*asg[ii].amplitude, factorFreq*asg[ii].freq, asg[ii].type);
  }

/* Fir coefficient array and length.
   Array may also be FIR_PASSTHRU (length = 0), to directly pass the input to output
   without filtering.
*/
void setFilter(uint16_t firFilt)
  {
  if (firFilt == LPF2300)           // LPF with 0 to 2300 Hz passband and -20 dB above.
    { // 100 KHz sample rate only
    firIn1.begin(lp2300_100K, 100);
    firIn2.begin(lp2300_100K, 100);
    }
  else if (firFilt == FILT_NONE)
    {
    firIn1.begin(FIR_PASSTHRU, 0);
    firIn2.begin(FIR_PASSTHRU, 0);
    }
  }

// setUpNewFreq(uint16_t nF)  -  All steps necessary to
// use a frequency.  fr is in Hz. In most cases this is ADC spur avoidance.
void setUpNewFreq(uint16_t nF)
  {
  // Converted to float ver .80
  float32_t fr = FreqData[nF].freqHz;
  topLines();
  if (fr < 2300.0f)
    {
    setSample(S100K);
    setFilter(LPF2300);
    }
  else if (fr < 3800.0f)
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  else if (fr < 5000.0f)
    {
    setSample(S48K);
    setFilter(FILT_NONE);
    }
  else if (fr < 12000.0f)
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  else if (fr < 13000.0f)
    {
    setSample(S48K);
    setFilter(FILT_NONE);
    }
  else if (fr < 20400.0f)
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  else if (fr < 21500.0f)
    {
    setSample(S96K);
    setFilter(FILT_NONE);
    }
  else if (fr < 28600.0f)
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  else if (fr < 29800.0f)
    {
    setSample(S96K);
    setFilter(FILT_NONE);
    }
  else if (fr < 37000.0f)
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  else if (fr < 38000.0f)
    {
    setSample(S96K);
    setFilter(FILT_NONE);
    }
  else
    {
    setSample(S100K);
    setFilter(FILT_NONE);
    }
  modifyFreq(fr);
  AudioNoInterrupts();
  waveform1.frequency(factorFreq * FreqData[nFreq].freqHz);
  waveform1.phase(0);
  waveform2.frequency(factorFreq * FreqData[nFreq].freqHz);
  waveform2.phase(270);
  AudioInterrupts();
  }

void print2serial(void)
  {
  float32_t normV, p;
  delay(50);
  //  if (annotate)
  //      Serial.print("Fr=");
  //  Serial.print(FreqData[nFreq].freqHz);
  // CSV output if not annotating
  if (uSave.lastState.ZorT == TRANSMISSION)
     {
     normV = amplitudeV / amplitudeR;   //ADD CAL
     if (annotate)  Serial.print(" Ampl=");
     else           Serial.print(",");
     // Scale up to be compatible with Serial.print
     Serial.print(normV, 4);
     if (annotate)  Serial.print(" Phase=");    // ADD CAL
     else           Serial.print(",");
     p = phaseV - phaseR - 180.0f;
     if (p > 180.0f)
        p = p - 360.0f;
     else if (p < -180.0f)
        p = p + 360.0f;
     Serial.println(p);
     }
  // Individual vectors for analysis - not normally needed
  if (verboseData)
     {
     Serial.print(" L & R Measurements V=");
     Serial.print(amplitudeV, 4);
     Serial.print("  ");
     Serial.print(phaseV, 2);
     Serial.print(" R=");
     Serial.print(amplitudeR, 4);
     Serial.print("  ");
     Serial.println(phaseR, 2);
     }
  printReady = false;
  }

// Use pins 2 and 3 to control panel 2-color LED
void panelLED(uint16_t what)
  {
  if (what == LSTART)           // Ready, but off
    {
    pinMode(PANEL_LED0, OUTPUT);
    pinMode(PANEL_LED1, OUTPUT);
    digitalWrite(PANEL_LED0, LOW);
    digitalWrite(PANEL_LED1, LOW);
    }
  else if (what == LGREEN)
    {
    digitalWrite(PANEL_LED0, LOW);
    digitalWrite(PANEL_LED1, HIGH);
    }
  else if (what == LRED)
    {
    digitalWrite(PANEL_LED0, HIGH);
    digitalWrite(PANEL_LED1, LOW);
    }
  else if (what == LOFF)
    {
    digitalWrite(PANEL_LED0, LOW);
    digitalWrite(PANEL_LED1, LOW);
    }
  }

/* Variable sample rate from Frank B., see
   https://forum.pjrc.com/threads/38753-Discussion-about-a-simple-way-to-change-the-sample-rate
   As pointed out by Frank, the various routines, like waveform generator, are calibrated
   for 44.11765 and need to be corrected when other sample rates. See factorFreq below.

   K12 Peripheral manual:
   MCLD Divide sets the MCLK divide ratio such that: MCLK output = MCLK input * ( (FRACT + 1) / (DIVIDE + 1) ).
   FRACT must be set equal or less than the value in the DIVIDE field.
   The MCLK divide ratio
   can be altered while an SAI is using that master clock, although the change in the divide
   ratio takes several cycles. MCR[DUF] can be polled to determine when the divide ratio
   change has completed.
*/

const uint16_t numFreqs = 9;
// sampleFreqs[] is of limited utility, as the index nSampleRate describes the selection,
// and sampleRateExact is the true rate, sometimes slightly different.
const int sampleFreqs[numFreqs] = {6000, 12000, 24000, 44100, 44117, 48000, 96000, 100000, 192000};
// Note Teensy 3.6:  F_CPU == 180000000, F_PLL == 180000000
// setI2SFreq(if) returns exact sample frequency, that may differ very slightly from sampleFreqs[]
double setI2SFreq(uint16_t iFreq)
{
  typedef struct
  {
    uint8_t mult;
    uint16_t div;
  } __attribute__((__packed__)) tmclk;
  // 44117 is nickname for 44117.64706

  if (F_PLL != 180000000)
    Serial.println("ERROR: Teensy 3.6 F_PLL should be 180MHz, but is not.");
  const tmclk clkArr[numFreqs] = {{16, 1875}, {32, 1875}, {64, 1875}, {196, 3125}, {16, 255}, {128, 1875}, {219, 1604}, {32, 225}, {219, 802}};
  /*  Info:
  #define I2S0_MCR          (*(volatile uint32_t *)0x4002F100) // SAI MCLK Control Register
  #define I2S_MCR_DUF       ((uint32_t)1<<31)                  // Divider Update Flag
  #define I2S_MCR_MOE       ((uint32_t)1<<30)                  // MCLK Output Enable
  #define I2S_MCR_MICS(n)   ((uint32_t)(n & 3)<<24)            // MCLK Input Clock Select
  #define I2S0_MDR          (*(volatile uint32_t *)0x4002F104) // SAI MCLK Divide Register
  #define I2S_MDR_FRACT(n)  ((uint32_t)(n & 0xff)<<12)         // MCLK Fraction
  #define I2S_MDR_DIVIDE(n) ((uint32_t)(n & 0xfff))            // MCLK Divide
   */
  while (I2S0_MCR & I2S_MCR_DUF) ;  // This is to make sure I2S controller is up to speed NEEDED??

  I2S0_MDR = I2S_MDR_FRACT((clkArr[iFreq].mult - 1)) | I2S_MDR_DIVIDE((clkArr[iFreq].div - 1));
  /* Serial.print("clkArr[iFreq].mult=");  Serial.println(clkArr[iFreq].mult);
     Serial.print("clkArr[iFreq].div=");  Serial.println(clkArr[iFreq].div);
     Serial.print("I2S0MDR=");  Serial.println(I2S0_MDR, HEX);
     Serial.print("F_PLL=");  Serial.println(F_PLL);  */
//rev.80
#define DOUBLE_256 ((double) 256.0L)
  return  ((double)F_PLL) * ((double)clkArr[iFreq].mult) / (DOUBLE_256 * ((double)clkArr[iFreq].div));
}

// valueStringSign()
// Same as valueString() but one char bigger(9) and adds blank or minus sign.
char *valueStringSign(float a, char *unitStr[])
  {
  static char str1[9];

  if(a < 0.0)
     str1[0] = '-';
  else
     str1[0] = ' ';
  strcpy(&str1[1], valueString(fabsf(a), &unitStr[0]));
  return str1;
  }

// Use this to create a print string for unsigned numbers
// In order to handle R, L and C values, the range is from
// 0.1 pico unit to 99 mega units (21 decades!). The units
// are set by the 2-char strings in unitString.  The resulting
// str1 is 7 char long plus a terminating zero.
char *valueString(float a, char *unitString[])
  {
  float aa;
  static char str1[8];
  char str0[8];
  strcpy(str1, "      ");
  // Print a to printf-type precision of 1 and four
  // significant digits, if possible.  Use leading spaces, if not.
  // 0.1 <= a < 1000.0
  if(fabsf(a) < 1.0E-13)
     {
     strcpy(str1, " 1/inf ");
     return str1;
     }
  if(a < 1.0E-9)       // The pF
     {
     aa = 1.0E12*a;
     sprintf(str0, "%.1f", aa);
     if(strlen(str0) <= 5)      // Add leading spaces
        {
        str1[5-strlen(str0)] = 0;
        strcat(str1, str0);
        }
     else
        {
        strcpy(str1, str0);
        }
     strcat(str1, unitString[0]);
     }
  else if(a < 1.0E-8)
     {
     aa = 1.0E9*a;                    // For C, 1 to 10 nF
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[1]);
     }
  else if(a < 1.0E-7)
     {
     aa = 1.0E9*a;                    // 10 to 100 nF
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[1]);
     }
  else if(a < 1.0E-6)
     {
     aa = 1.0E9*a;                    // 100 to 1000 nF
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[1]);
     }

  else if(a < 1.0E-5)
     {
     aa = 1.0E6*a;                    // 1 to 10 uF
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[2]);
     }
  else if(a < 1.0E-4)
     {
     aa = 1.0E6*a;                    // 10 to 100 uF
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[2]);
     }
  else if(a < 1.0E-3)
     {
     aa = 1.0E6*a;                    // 100 to 1000 uF
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[2]);
     }
  else if(a < 1.0E-2)
     {
     aa = 1.0E3*a;                    // 1000 to 10,000 uF
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[3]);
     }
  else if(a < 0.1)
     {
     aa = 1.0E3*a;                    // 10,000 to 100,000 uF
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[3]);
     }
  else if(a < 1.0)
     {
     aa = 1.0E3*a;                    // 0.1 to 1
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[3]);
     }
  else if(a < 10.0)
     {
     aa = a;                    // 1 to 10
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[4]);
     }
  else if(a < 100.0)
     {
     aa = a;                    // 10 to 100
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[4]);
     }
  else if(a < 1000.0)
     {
     aa = a;                    // 100 to 1000
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[4]);
     }
  else if(a < 10000.0)
     {
     aa = 1.0E-3*a;                    // 1000 to 10,000
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[5]);
     }
  else if(a < 1.0E5)
     {
     aa = 1.0E-3*a;                    // 10,000 to 100,000
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[5]);
     }
  else if(a < 1.0E6)
     {
     aa = 1.0E-3*a;                    // 100,000 to 1.0E6
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[5]);
     }
  else if(a < 1.0E7)
     {
     aa = 1.0E-6*a;                    // 1.0E6 to 1.0E7
     sprintf(str1, "%.3f", aa);
     strcat(str1, unitString[6]);
     }
  else if(a < 1.0E8)
     {
     aa = 1.0E-6*a;                    // 1.0E7  to 1.0E8
     sprintf(str1, "%.2f", aa);
     strcat(str1, unitString[6]);
     }
  else if(a<1.0E9)
     {
     aa = 1.0E-6*a;                    // 1.0E7  to 1.0E8
     sprintf(str1, "%.1f", aa);
     strcat(str1, unitString[6]);
     }
  else
     strcpy(str1, "  ***  ");         // It can happen for X at 40 KHz
  return str1;
  }
