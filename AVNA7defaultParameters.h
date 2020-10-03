#if 0
// This structure needs default values, just to get started and stored as defaults in EEPROM. 
// Many of these will be revised after things are working.
// Revised for adding data formats.  59 decimal = 0X3B for first byte (was originally 0X21)
// SHOWN HERE FOR REFERENCE ONLY---the real structure is in the main program.
struct saveState {
  uint16_t startup;        // Detect valid EEPROM. startup=0X433B, first byte 0X3B, second 0X43 always
  float    freqHz0;        // Goes with nFreq = 0
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
  // Next four set the data formats.  See LinLogCommand()  rev 0.5.9
  uint8_t  rsData;
  uint8_t  tsData;
  uint8_t  rdData;
  uint8_t  tdData;
  // Save calibration for 13 quasi-log points (added rev 0.70):
  double   EvRatio[13];            // Vor/Vom magnitudes
  double   EdPhase[13];            // phaseor - phaseom
  double   EthruRefAmpl[13];
  double   EthruRefPhase[13];
  uint8_t  int8version;
  // Constants for VVM, Spec Analyzer, Sig Gens and Cal functions.  Added at rev 0.80
  // TFT corner values, settable from Touch Sceen Cal.
  uint16_t xMin;   // = TS_MINX;  // 320
  uint16_t yMin;   // = TS_MINY;  // 320
  uint16_t xMax;   // = TS_MAXX;  // 3770
  uint16_t yMax;   // = TS_MAXY;  // 3770
  float32_t sgCal; // = 1.5792f;
  float32_t VVMCalConstant;    // = 0.0000137798f;
  float32_t SAcalCorrectionDB; // = 4.5f;
};
#define DEFAULT_PARAMETERS {0X433B, 1000.0, 999.9736, 1, IMPEDANCE, SWEEP, R50,
{0.1, 50.00, 5000.0}, 1.0, 40.0E-12, 1000000.0, 0.22E-6, 0.07, 20.0E-9, 2, 1, 2, 0,
{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
CURRENT_VERSION, 320, 320, 3770, 3770, 1.5792f, 0.0000137798f, 4.5f}
// End reference copy, human readable but not proper #define - NOT USED
#endif
 
// What follows is the same, all on one line, ready to be used (no semi colon at end here)
#define DEFAULT_PARAMETERS {0X433B, 1000.0, 999.9736, 1, IMPEDANCE, SWEEP, R50, {0.1, 50.00, 5000.0}, 1.0, 40.0E-12, 1000000.0, 0.22E-6, 0.07, 20.0E-9, 2, 1, 2, 0,{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, CURRENT_VERSION, 320, 320, 3770, 3770, 1.5792f, 0.0000137798f, 4.5f}
