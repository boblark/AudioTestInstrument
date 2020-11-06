// Serial Commands for AVNA  Made a module 24 Jan 2020  RSL
/*  RSL_VNA8 Arduino sketch for audio VNA measurements.
 *  Copyright (c) 2016-2020 Robert Larkin  W7PUA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
// ---------------------  xxxxCommand()  Responses to serial commands  -------------

// INSTRUMENT command
// Defines the Instrument, these are mutually exclusive
// ALL_IDLE -9, AVNA 0, VVM  1, ASA  2, VIN_CAL 3, VOUT_CAL 4, TOUCH_CAL 5
void InstrumentCommand(void)
  {
  char *arg;

  doRun = RUNNOT;  // Stop any measurements

  arg = SCmd.next();
  if (arg != NULL)
    {
    if (atoi(arg) == ALL_IDLE)
      {
      instrument = ALL_IDLE;
      doRun = RUNNOT;
      nRun = 0;
      tToInstrumentHome();
      writeMenus(0);  

      if (verboseData)  Serial.println("Set to All Idle");
      }
    else if (atoi(arg) == AVNA)
      {
      instrument = AVNA;
      doRun = RUNNOT;
      nRun = 0;
      tToAVNAHome();
      writeMenus(1);  
      if (verboseData)  Serial.println("Set to AVNA");
      }
    else if (atoi(arg) == VVM)
      {
      instrument = VVM;
      doRun = RUNNOT;
      nRun = 0;
      tToVVM();
      writeMenus(17);  
      if (verboseData)  Serial.println("Set to Vector Voltmeter");
      }
    else if (atoi(arg) == ASA)
      {
      instrument = ASA;
      doRun = RUNNOT;
      nRun = 0;
      if (verboseData)  Serial.println("Set to Spectrum Analyzer");
      tToASA();
      prepSpectralDisplay();
      writeMenus(12);  
      }
    else if (atoi(arg) == VIN_CAL)
      {
      instrument = VIN_CAL;
      if (verboseData)  Serial.println("Set to VIn Calibrate");
      writeMenus(20);  
      tVInCal();
      }
    else if (atoi(arg) == VOUT_CAL)
      {
      instrument = VOUT_CAL;
      if (verboseData)  Serial.println("Set to Vout Calibrate");
      writeMenus(21);  
      tVOutCal();
      }
    else if (atoi(arg) == TOUCH_CAL)
      {
      instrument = TOUCH_CAL;
      if (verboseData)  Serial.println("Set to Touch Calibrate");
      writeMenus(19);  
      tTouchCal();
      }
    }
  }

/* SIGGEN i s f a w  Command
  i=index 1,4;  s=On/Off 1,0;  f=freq Hz; a=ampliitude 0.0, 1.0;
  For w, see https://www.pjrc.com/teensy/gui/?info=AudioSynthWaveform
  WAVEFORM_SINE              0
  WAVEFORM_SAWTOOTH          1
  WAVEFORM_SQUARE            2
  WAVEFORM_TRIANGLE          3
  WAVEFORM_SAWTOOTH_REVERSE  6
  This can be set anytime and is not associated with a particular instrument.
  This command may not LCD display any change. The noise generator, SG #4, 
  only reponds to on/off and amplitude (1-sigma value).  */
void SigGenCommand(void)
  {
  char *arg;
  uint16_t sgIndex;

  arg = SCmd.next();
  if (arg != NULL)
    {
    int ii = atoi(arg);
    if(ii>0 && ii<=4)
      sgIndex = ii - 1;
    else
      return;
    }
  else
    return;

  arg = SCmd.next();
  if (arg != NULL)
    {
    asg[sgIndex].ASGoutputOn = (atoi(arg) != 0);  // bool
    }

  arg = SCmd.next();
  if (arg != NULL && sgIndex != 3)  // not for noise
    {
	float ff = (float)atof(arg);
    asg[sgIndex].freq = ff;
    if (ff < 0.0f  ||  ff > 0.5f*sampleRateExact)
      asg[sgIndex].ASGoutputOn = false;
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
	float aa = (float)atof(arg);
	if(aa < 0.0f)  aa = 0.0f;
	if(aa > 1.0f)  aa = 1.0f;
    asg[sgIndex].amplitude = aa;
    }

  arg = SCmd.next();
  if (arg != NULL && sgIndex != 3)
    {
    int w = atoi(arg);
    if (w==0 || w==1 || w==2 || w==3 || w==4 || w==6)
      asg[sgIndex].type = w;
    else
      asg[sgIndex].type = 0;
    }
  if (verboseData)
     {
     if(sgIndex==3)
       Serial.print("Noise Generator, SG ");
     else
	   Serial.print("Sig Gen ");
	 Serial.print(sgIndex);
	 Serial.println(" successfully programmed.");
     }
  if(currentMenu == 15)  tToASGHome();  // Redraw
  if(currentMenu == 16)  tToASGn();
  checkSGoverload();
  setSigGens();
  }



// Command the VVM on, measure continuous unless stopped by "RUN -2"
//   Continuous: "RUN 0"
//   n Measurements over Serial: "RUN n"
//   Stop measure over Serial:  "RUN -2"
void VVMCommand(void)
  {
  char *arg;

  arg = SCmd.next();
  if (arg != NULL)
    {
    VVMSendSerial = (atoi(arg) != 0);  // bool
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
    int u = atoi(arg);                  // 2 possibilities
    if (u==1)  VVMFormat = VVM_VOLTS;
    if (u==2)  VVMFormat = VVM_DBM;
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
	float p = (float)atof(arg);
    if(p>180.0)        phaseOffset =  180.0;
    else if(p<-180.0)  phaseOffset = -180.0;
    else               phaseOffset = p;
    }

  if (verboseData)  Serial.println("VVM successfully programmed.");

  if(instrument == VVM)
    {
    nRun=0;    // Continuous
    doRun = CONTINUOUS;
    tToVVM();  // Setup screen, etc
    writeMenus(17);  
    }
  }

/* Command the Spectrum Analyzer on
 * ASACommand fmt sr m d of
 *  fmt = Serial Format
 *   sr = sample rate, 0 to 6
 *    m = number of averages > 0
 *    d = dB/div e.g., 10.0
 *   of = offset in dB, e.g. 12.5
 */
void ASACommand(void)
  {
  char *arg;

  instrument = ASA_IDLE;
/*  arg = SCmd.next();      // Not Needed??
  if (arg != NULL)
    {
    ASASendSerial = (atoi(arg) != 0);  // bool
    }
 */

/*  1  = Comma dividers
 *  2  = Space Dividers (can be after comma)
 *  4  = Column of 512 (0 is row of 512)
 *  8  = CR-LF not just LF (for columns)
 * 16  = Leading/Trailing '|'
 * 32  = dBm, not numerical power          */
  arg = SCmd.next();
  if (arg != NULL)
    ASASerialFormat = (uint16_t)atoi(arg);

  /* ASAI2SFreqIndex = 0  6 KHz Sample Rate
   *                   1 12 kHz Sample Rate
   *                   2 24 kHz Sample Rate
   *                   3 48 kHz Sample Rate
   *                   4 96 kHz Sample Rate  */
  arg = SCmd.next();
  if (arg != NULL)
    {
    int sr = atoi(arg);
    if (sr>=0 && sr<=6)
      {
      ASAI2SFreqIndex = sr;
      setSample(freqASA[ASAI2SFreqIndex].rateIndex);
      countMax = freqASA[ASAI2SFreqIndex].SAnAve;  // Anyone use this??? <<<<<<<<<<<<<<<<<<
      }
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
    int m = atoi(arg);
    // NOTE - Set sample rate index above. Therefore next item
    // applies ONLY to this one index
    if(m>0 && m<1000)
      {
      freqASA[ASAI2SFreqIndex].SAnAve = m;
      setSample(freqASA[ASAI2SFreqIndex].rateIndex);
      countMax = freqASA[ASAI2SFreqIndex].SAnAve;
      }
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
    float d = atof(arg);
    if(d>0.1f  &&  d<100.0f)
      dbPerDiv = d;
    }

  arg = SCmd.next();
  if (arg != NULL)
    {
    float of = atof(arg);
    if(of>=-200.0f  &&  of<=200.0f)
      ASAdbOffset = of;
    }

  instrument = ASA;

  if (verboseData)  Serial.println("ASA successfully programmed.");

  nRun =  RUNNOT;    // Wait for RUNn trigger
  doRun = CONTINUOUS;
  prepSpectralDisplay();
  tToASA();  // Setup screen, etc
  writeMenus(12);
  }

void ScreenSaveCommand(void)
  {
  char *arg;
  bool printFilename = false;

  arg = SCmd.next();
  if (arg != NULL)
    printFilename = (atoi(arg) != 0);  // bool

  if(SDCardAvailable)
    {
    bmpScreenSDCardRequest = true;  // Do anything useful? It is needed!
    char* pf = dumpScreenToSD();
    if(verboseData || printFilename)
      {
      Serial.print("Screen Save: ");
      Serial.println(pf);
      }
    }
  }

// TRANSMISSION command
void TransmissionCommand()
  {
  char *arg;
  doingNano = false;
  doRun = RUNNOT;  // Stop any measurements
  uSave.lastState.ZorT = TRANSMISSION;

  arg = SCmd.next();
  if (arg != NULL)
    {
    if (atoi(arg) == 50)
      {
      uSave.lastState.iRefR = R50;
      }
    else if (atoi(arg) == 5000)
      {
      uSave.lastState.iRefR = R5K;
      }
    if (verboseData)
      {
      Serial.print("Transmission measure; Ref R = ");
      Serial.println(uSave.lastState.valueRRef[uSave.lastState.iRefR], 2);
      }
    }
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  topLines();
  }


// FreqCommand changes the single frequency being used for serial control.  This is saved
// in   uSave.lastState.freqHz0  to endure shut down.  At startup, it will
// be moved also to FreqData[0] to allow calibration.
// This automatically sets the single frequency operation (non-sweep).
void FreqCommand(void)
  {
  char *arg;
  arg = SCmd.next();
  doingNano = false;
  doRun = RUNNOT;  // Stop any measurements
  if (arg != NULL)
    {
    uSave.lastState.freqHz0 = (float)atof(arg);
    FreqData[0].freqHz = uSave.lastState.freqHz0;
    nFreq = 0;                   // For single frequency operation
    // setUpNewFreq(0);  Was b4 rev.70
    prepMeasure(FreqData[0].freqHz); //Rev0.70 this sets all of FreqData[0].zzzz
    uSave.lastState.SingleorSweep = SINGLE;
    // saveStateEEPROM();  rev0.70 cut down on EEPROM writes; use SAVE command
    if (verboseData)
      {
      Serial.print("New Single Frequency: ");
      Serial.println(uSave.lastState.freqHz0);
      // Serial.println(" - Needs cal");  rev0.70 remove
      }
    }                 // ELSE ERROR RESPONSE
  topLines();
  }

// SweepCommand sets up sweep of fixed table of frequencies.
// Parallels FreqCommand().  Use CAL and RUN to complete
void SweepCommand(void)
  {
  doingNano = false;
  doRun = RUNNOT;  // Stop any measurements
  nFreq = 1;                   // For sweep
  uSave.lastState.SingleorSweep = SWEEP;
  saveStateEEPROM();
  if (verboseData)
    Serial.println("Sweep data setup"); // rev0.70 was - needs cal.");
  topLines();
  }

void CalCommand(void)
  {
  CalCommand0(1);
  }

// CAL command.  If Single freq, does that freq of cal.
// If sweep, does standard logarithmic set.
// baseI allows for leaving out bottom frequencies.
// baseI = 1 for all freq
void CalCommand0(uint16_t baseI)
  {
  clearStatus();
  doingNano = false;
  doRun = RUNNOT;  // Stop any measurements
  if(verboseData && !doTuneup)  Serial.print("Doing CAL");
  if (uSave.lastState.ZorT == IMPEDANCE)
    {
    if(verboseData && !doTuneup)  Serial.println(" of Impedance.");
    if (uSave.lastState.SingleorSweep == SINGLE)
      {
      setUpNewFreq(nFreq);
      setRefR(R_OFF);         // Disconnect relay connection to Z terminal
      setSwitch(CAL_39);      // FST3125 switch connection to both measure and reference channels
      if(!doTuneup)           // Tuneup manages this itself to avoid excess delays during repititions
        {
        DC1.amplitude(dacLevel);     // Turn on sine wave
        delay(50);
        }
      getFullDataPt();
      checkOverload();
      // These are in polar form.  FreqData[nFreq].vRatio is a minor correction for
      // differences in the gain of V and R channels. dPhase is same but for phase.
      FreqData[nFreq].vRatio = amplitudeR / amplitudeV;  // For use later
      FreqData[nFreq].dPhase = phaseR - phaseV;          // For use later
      if(!doTuneup)
        {
        setRefR(uSave.lastState.iRefR);           // Restore relay settings
        printCalDetails();
        }
      calZSingle = true;
      if(!doTuneup)                 // No bunches of "C" for Tuneup
        {
        if(verboseData)
           Serial.println("...Cal complete.");
        else
           Serial.println("C");
        }
      }
    else if (uSave.lastState.SingleorSweep == SWEEP)  // Impedance Sweep
      {
      setRefR(uSave.lastState.iRefR);
      setSwitch(CAL_39);
      DC1.amplitude(dacLevel);     // Turn on sine wave
      delay(50);
      for (nFreq = baseI; nFreq <= 13;  nFreq++) // Over all frequencies; nFreq is global freq index
        {
        if(verboseData) Serial.print(nFreq);
        setUpNewFreq(nFreq);
        delay(ZDELAY);   // Delay until level is constant
        getFullDataPt();
        checkOverload();
        FreqData[nFreq].vRatio = amplitudeR / amplitudeV;  // For use later
        FreqData[nFreq].dPhase = phaseR - phaseV;          // For use later
        //  CHECK FOR WRAP-AROUND???     <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        printCalDetails();
        }
      calZSweep = true;
      if(verboseData)  Serial.println("C");
      }
    }
  else             // Transmission
    {
    // Cal here is in two parts.  First the vRatio/dPhase correction
    // for the ADC input circuitry.  Then a second correction to normalize the
    // measurement
    if(verboseData)  Serial.print(" of Transmission. A reference thru-path is needed.");
    if (uSave.lastState.SingleorSweep == SINGLE)
      {
      setUpNewFreq(nFreq);

      // Get numbers for first correction
      setRefR(R_OFF);         //  Disconnect the output
      setSwitch(CAL_39);      // Drive both ADC channels together
      delay(200);
      getFullDataPt();
      checkOverload();
      // These are in polar form.  FreqData[nFreq].vRatio is the voltage at the
      // measurement input. dPhase is the measured phase.  Small difference between
      // the 2 ADC channels.
      FreqData[nFreq].vRatio = amplitudeR / amplitudeV;
      FreqData[nFreq].dPhase = phaseR - phaseV;

      // And for second correction
      // (for low Z output, manual delay here) <<<<<<<<<<<<<<<<<<<<<<<<
      setRefR(uSave.lastState.iRefR);   // Connect to 50 or 5K ohm output
      setSwitch(TRANSMISSION_37);      // Thru path
      delay(200);
      getFullDataPt();
      //if (printReady)
      //   print2serial();
      checkOverload();
      // The through path voltage gain, using first correction
      FreqData[nFreq].thruRefAmpl = (amplitudeV / amplitudeR) * FreqData[nFreq].vRatio;
      FreqData[nFreq].thruRefPhase = phaseV - phaseR + FreqData[nFreq].dPhase;
      printCalDetails();
      calTSingle = true;
      if(verboseData)
         Serial.println("...Cal complete.");
      else
         Serial.println("C");
      }
    else           // Swept transmission cal
      {
      setUpNewFreq(1);
      // Get numbers for first correction
      setRefR(R_OFF);         //  Disconnect the output
      setSwitch(CAL_39);      // Drive both ADC channels together
      delay(200);
      for (nFreq = baseI; nFreq <= 13;  nFreq++) // Over all freqs; nFreq is global freq index
        {
        if (verboseData)  Serial.print(nFreq);
        setUpNewFreq(nFreq);
        delay(100);            // To be sure
        getFullDataPt();
        checkOverload();
        FreqData[nFreq].vRatio = amplitudeR / amplitudeV;
        FreqData[nFreq].dPhase = phaseR - phaseV;
        }
      if (verboseData)  Serial.println("");
      // And for second correction
      // (for low Z output, manual delay here) <<<<<<<<<<<<<<<<<<<<<<<<
      setRefR(uSave.lastState.iRefR);   // Connect to 50 or 5K ohm output
      setSwitch(TRANSMISSION_37);      // Thru path
      delay(200);
      for (nFreq = baseI; nFreq <= 13;  nFreq++) // Over all freq; nFreq is global freq index
        {
        // if (verboseData)  Serial.print(13 + nFreq);   rev 0.6.0
        setUpNewFreq(nFreq);
        delay(100);
        getFullDataPt();
        checkOverload();
        // The through path voltage gain, using first correction
        FreqData[nFreq].thruRefAmpl = (amplitudeV / amplitudeR) * FreqData[nFreq].vRatio;
        FreqData[nFreq].thruRefPhase = phaseV - phaseR + FreqData[nFreq].dPhase;
        printCalDetails();
        }
      calTSweep = true;
      if(verboseData)
         Serial.println("...Cal complete.");
      else
         Serial.println("C");
    }
  }
}          // End CalCommand

// Verbose printing for Cal
void printCalDetails(void)
  {
  if(verboseData)
      {
      Serial.print("Fr=");       Serial.print(FreqData[nFreq].freqHz, 3);
      Serial.print("  V=");      Serial.print(amplitudeV);
      Serial.print(" Vph=");     Serial.print(phaseV);
      Serial.print("  VR=");      Serial.print(amplitudeR);
      Serial.print(" VRph=");     Serial.print(phaseR);
      Serial.print("  vRatio="); Serial.print(FreqData[nFreq].vRatio, 5);
      Serial.print(" dPhase=");  Serial.println(FreqData[nFreq].dPhase);
      }
  }

// Do a Cal at 13 standard points and save it to EEPROM.  First parameter:
//   0 = Impedance
//   1 = Transmission
//   2 = Readback of stored Cal
// Second parameter is the number of points to average, default 1.
// This is used manually to make measurements more convenient by using this
// default cal.z
void CalSaveCommand()
  {
  char *arg;
  uint16_t doWhat, i, k;
  uint16_t nAve = 1;
  double t1, t2;    // Temporary storage
  doingNano = false;
  arg = SCmd.next();
  if (arg != NULL)
    doWhat = (uint16_t)atoi(arg);
  else
    {
    Serial.println("No parameter was found for CALSAVE");
    return;
    }

  // The second parameter is the number of points to average, default 1. (implement<<<)
  arg = SCmd.next();
  if (arg != NULL)
    {
    nAve = (uint16_t)atoi(arg);
    Serial.print("Number of points averaged: ");
    Serial.println(nAve);
    }

  if(doWhat == 0)
    {
    uSave.lastState.SingleorSweep = SINGLE;
    nanoState = NO_NANO;  // Cancel if in place
    doRun = RUNNOT;  // Stop any measurements
    Serial.println("Wait - Doing default impedance calibration for EEPROM save.");
    uSave.lastState.ZorT = IMPEDANCE;
    for (nFreq=1; nFreq<=13; nFreq++)
      {
      t1 = 0.0;  t2 = 0.0;
      for (k=1; k<=nAve; k++)
        {
        CalCommand();
        t1 += FreqData[nFreq].vRatio;
        t2 += FreqData[nFreq].dPhase;
        }
      uSave.lastState.EvRatio[nFreq] = t1/(double)nAve;
      uSave.lastState.EdPhase[nFreq] = t2/(double)nAve;
      }
      saveStateEEPROM();
      Serial.println("Impedance Cal and save to EEPROM complete.");
    }

  else if (doWhat == 1)
    {
    uSave.lastState.SingleorSweep = SINGLE;
    doRun = RUNNOT;  // Stop any measurements
    Serial.println("Wait - Doing default transmission calibration for EEPROM save.");
    uSave.lastState.ZorT = TRANSMISSION;
    for (nFreq=1; nFreq<=13; nFreq++)
      {
      t1 = 0.0;  t2 = 0.0;
      for (k=1; k<=nAve; k++)
        {
        CalCommand();
        t1 += FreqData[nFreq].thruRefAmpl;
        t2 += FreqData[nFreq].thruRefPhase;
        }
      uSave.lastState.EthruRefAmpl[nFreq] = t1/(double)nAve;
      uSave.lastState.EthruRefPhase[nFreq] = t2/(double)nAve;
      }
      saveStateEEPROM();
      Serial.println("Transmission Cal and save to EEPROM complete.");
    }
  else
    {
    Serial.println("Current calibration, gain and phase:");
    for(i=0; i<=13; i++)
      {
      if(i == 0)
         Serial.println("General data point (index=0):");
      if(i == 1)
         Serial.println("Standard data points, 10 to 40,000 Hz:");
      Serial.print  (FreqData[i].vRatio,6);      Serial.print(" ");
      Serial.print  (FreqData[i].dPhase,3);      Serial.print(" ");
      Serial.print  (FreqData[i].thruRefAmpl,6); Serial.print(" ");
      Serial.println(FreqData[i].thruRefPhase,3);
      }
    }
  }

// RunCommand, in the command,  takes a parameter n that means to take n single measurements
// or to do n sweeps.  An zero value for n is to never stop (except with "RUN n" with n>0).
// -1 is special single measure without cal. -2 or less is no run
// The command is either "RUN n" or "R n".
//      doRun:  RUNNOT, COUNTDOWN, CONTINUOUS, SINGLE_NC, POWER_SWEEP (single serial meas, no cal)
//                0         2           1          3          4
//      nRun: -1=stop measuring, 0 measure continuous, and 0<n is remaining measurements
// rev .82 Use RUN command to control VVM and ASA. Same deal:  0=continuous, n>0 do n measurements
void RunCommand(void)
  {
  char dr[16];
  char *arg;
  doingNano = false;
  arg = SCmd.next();
  if (arg != NULL)
    {
    nRun = atoi(arg);
    if (nRun == 0)
      {
      strcpy(dr, " (Continuous)");
      doRun = CONTINUOUS;
      }
    else if (nRun > 0)
      {
      doRun = COUNTDOWN;
      strcpy(dr, " (Countdown)");
      }
    else if(nRun == -1)
      {
      doRun = SINGLE_NC;
      strcpy(dr, " (Single)");
      }
    else  // -2 or less
      {
      doRun = RUNNOT;
      nRun = 0;
      strcpy(dr, " (Not measuring)");
      }
    if (verboseData)
      {
      Serial.print("Set RUN n = ");
      Serial.print(nRun);
      Serial.println(dr);
      }
    }
  }

//  NOT COMPLETE   <<<<<<<<<<<<<<<<<<
void PowerSweepCommand(void)
  {
  doingNano = false;
  doRun = POWER_SWEEP;
  dacSweep = 0.01;   // Level during sweep
  Serial.println("Starting Power Sweep");
  }

// setRefR() controls the two relays to determine the reference resistance
// for impedance measurements.  R_OFF. R50 or R5K
void setRefR(uint16_t refState)
{
  if (refState == R_OFF)
  {
    digitalWrite(R50_36, LOW);        // Off
    digitalWrite(R5K_35, LOW);        // Off
  }
  else if (refState == R50)
  {
    digitalWrite(R50_36, HIGH);        // On
    digitalWrite(R5K_35, LOW);
  }
  else if (refState == R5K)
  {
    digitalWrite(R50_36, LOW);
    digitalWrite(R5K_35, HIGH);         // On
  }
}

// setSwitch(uint16_t what) Sets the measurement channel to
// CAL_39, IMPEDANCE_38 or TRANSMISSION_37.
void setSwitch(uint16_t what)
  {
  if (what == CAL_39)
    {
    digitalWrite(CAL_39, HIGH);          // On
    digitalWrite(IMPEDANCE_38, LOW);     // Off
    digitalWrite(TRANSMISSION_37, LOW);  // Off
    }
  else if (what == IMPEDANCE_38)
    {
    digitalWrite(CAL_39, LOW);             // Off
    digitalWrite(IMPEDANCE_38, HIGH);      // On
    digitalWrite(TRANSMISSION_37, LOW);    // Off
    }
  else if (what == TRANSMISSION_37)
    {
    digitalWrite(CAL_39, LOW);             // Off
    digitalWrite(IMPEDANCE_38, LOW);       // Off
    digitalWrite(TRANSMISSION_37, HIGH);   // On
    }
  }

// ZMEAS Command - Only sets up Z measurements
void ZmeasCommand()
  {
  char *arg;
  // Complex RR50(uSave.lastState.valueRRef[1], 0.0);
  // Complex RR5K(uSave.lastState.valueRRef[2], 0.0);
  doingNano = false;
  doRun = RUNNOT;  // Stop any measurements
  uSave.lastState.ZorT = IMPEDANCE;
  arg = SCmd.next();
  if (arg != NULL)
    {
    if (atoi(arg) == 50)
       uSave.lastState.iRefR = R50;
    else if (atoi(arg) == 5000)
       uSave.lastState.iRefR = R5K;
    if (verboseData)
      {
      Serial.print("Set Ref R = ");
      Serial.println(uSave.lastState.valueRRef[uSave.lastState.iRefR]);
      }
    topLines();
    }
  }

void DelayCommand()
{
  char *arg;
  arg = SCmd.next();
  if (arg != NULL)
  {
    uSave.lastState.msDelay = (uint16_t)atoi(arg);  // New loop delay
    saveStateEEPROM();
    if (verboseData)
    {
      Serial.print("New Loop Delay: ");
      Serial.println(uSave.lastState.msDelay);
    }
  }                 // ELSE ERROR RESPONSE
}


//  From CALDAT command.
void CalDatCommand(void)
  {

  }

// From SERPAR command.  Sets type of output data for Z meas
//    SERPAR s p  s sets serial output, p sets parallel output choice
void SerParCommand(void)
  {
  char *arg;
  // First parameter after SERPAR
  arg = SCmd.next();  // Returns ptr to null terminated string, parse series RX
  if (arg == NULL)
    return;
  else if (strcmp(arg, "0") == 0)
    {
    seriesRX = false;
    if(annotate)
      Serial.println("Series RX not outputted");
    }
  else if (strcmp(arg, "1") == 0)
    {
    seriesRX = true;
    if(annotate)
      Serial.println("Series RX outputted");
    }

  // p for parallel options
  arg = SCmd.next();      // now parse parallel RX desires
  if (arg == NULL)
    return;
  else if (strcmp(arg, "0") == 0)
    {
    parallelRX = false;
    if(annotate)
      Serial.println("Parallel RX not outputted");
    }
  else if (strcmp(arg, "1") == 0)
    {
    parallelRX = true;
    if(annotate)
      Serial.println("Parallel RX outputted");
    }
  // Add possibility of S11 and S21 output?
  }

void TestCommand(void)
  {
  char *arg;

  teststate = 1;
  arg = SCmd.next();
  if (arg != NULL)
    {
    test_ry = (uint16_t)atoi(arg);
    }
  arg = SCmd.next();
  if (arg != NULL)
    {
    test_sw = (uint16_t)atoi(arg);
    }
    digitalWrite(CAL_39, LOW);           // Off
    digitalWrite(IMPEDANCE_38, LOW);     // Off
    digitalWrite(TRANSMISSION_37, LOW);  // Off
 // TEST 0 0  will leave all three switches off
 if(test_sw & 0X0001)
     digitalWrite(CAL_39, HIGH);          // On
 if(test_sw & 0X0002)
    digitalWrite(IMPEDANCE_38, HIGH);     // On
 if(test_sw & 0X0004)
    digitalWrite(TRANSMISSION_37, HIGH);  // On
 setRefR(test_ry);

 DC1.amplitude(dacLevel);     // Turn on sine wave
 setUpNewFreq(7);         // 1000 Hz

  Serial.print("Test Mode -  RY = ");
  Serial.print(test_ry);
  Serial.print("  Test SW = ");
  Serial.println(test_sw);
  Serial.println("Restart required to reset to ordinary measurements.");
  }

// Param1Command()  -  "PARAM1 0 50.22 5017.3"
// The '0' means "no default."  Set to the number 99 and ALL EEPROM values will be set to the
// defaults.  For this reset, the next two parameters are ignored, and are not required.
// A value 0 causes the reference resistor values to be set to the two parameter values.
// PARAM1 with no following parameters  prints the two reference resistor values.
void Param1Command(void)
  {
  char *arg;

  arg = SCmd.next();
  if(arg == NULL)
     {
     Serial.println("Currrent PARAM1 values:");
     Serial.print("  50 Ohm reference  = "); Serial.println(uSave.lastState.valueRRef[1], 3);
     Serial.print("  5K Ohm reference  = "); Serial.println(uSave.lastState.valueRRef[2], 1);
     return;
     }
  else if(atoi(arg) == 99)
     {
     if(annotate)  Serial.println("Full reset of parameters!");
     uSave.lastState = DEFAULT_PARAMETERS;
     saveStateEEPROM();
     topLines();
     return;
     }
  else if(atoi(arg) != 0)
     return;
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.valueRRef[1] = atof(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.valueRRef[2] = atof(arg);
  saveStateEEPROM();
  topLines();
  }

/* param2Command() - Serves the changing of correction factors for the impedance measurements.
 * arg in order:     capInput resInput capCouple seriesR seriesL
 *             "PARAM2 37.0  1000000.0    0.22    0.07   20.0"
 * With units           pF      Ohm        uF      Ohm    nH
 *
 * Note capInput is the most likely item to change, and it can be changed with simply "PARAM2 34.8"
 * To obtain current values, type  PARAM2  with no parameters
 */
void Param2Command(void)
  {
  char *arg;

  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.capInput = 1.0E-12 * atof(arg);
  else
     {
     Serial.println("Currrent PARAM2 values:");
     Serial.print("  Input cap  = "); Serial.println(1.0E12 * uSave.lastState.capInput, 1);
     Serial.print("  Input res  = "); Serial.println(uSave.lastState.resInput, 0);
     Serial.print("  Couple cap = "); Serial.println(1.0E6 * uSave.lastState.capCouple, 3);
     Serial.print("  Series res = "); Serial.println(uSave.lastState.seriesR, 4);
     Serial.print("  Series ind = "); Serial.println(1.0E9 * uSave.lastState.seriesL, 1);
     return;
     }
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.resInput = atof(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.capCouple = 1.0E-6 * atof(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.seriesR = atof(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.seriesL = 1.0E-9 * atof(arg);
  arg = SCmd.next();
  saveStateEEPROM();
  }

// Command for providing lots of feedback
void VerboseCommand()
  {
  char *arg;
  arg = SCmd.next();      // Returns ptr to null terminated string
  if (arg == NULL)
    return;
  else if (strcmp(arg, "0") == 0)
    {
    verboseData = false;
    if(annotate)
      Serial.println("Verbose off");
    }
  else if (strcmp(arg, "1") == 0)
    {
    verboseData = true;
    if(annotate)
      Serial.println("Verbose enabled");
    }
  }

// Tuneup is used at setup time, or whenever parameters need to be checked.  These are
// the values that correct for circuit "stray" components to improve the accuracies at
// impedances near the extreme values.  This is available only over the serial port and
// is directed manually, since components, opens and shorts need to be connected.
// NOTE:  This is EXPERIMENTAL. In general, the results are better by using externally
// calculated values, except especially Cin.  Use this command with care and remember that
// default values are available from the serial command "PARAM1 99".
void TuneupCommand(void)
  {
  static float eRs,     eLs,     eR50,     eR5K,     e1Meg,     eCin;
  static float eRsSave, eLsSave, eR50Save, eR5KSave, e1MegSave, eCinSave;
  float meas1, meas2, meas3, meas4;
  float rPrecise;
  uint16_t ju;
  char *arg;
  static boolean beenHere = false;
  Complex Zmeas(0.0, 0.0);
  Complex Ymeas(0.0, 0.0);
  Complex Vm(0.0, 0.0);
  Complex Vr(0.0, 0.0);

  // A copy for "go back", saved first time tuneupCommand() is run
  if (!beenHere)
    {
    eRsSave=  uSave.lastState.seriesR;        eLsSave= uSave.lastState.seriesL;
    eR50Save= uSave.lastState.valueRRef[R50]; eR5KSave=uSave.lastState.valueRRef[R5K];
    e1MegSave=uSave.lastState.resInput;       eCinSave=uSave.lastState.capInput;
    }
  beenHere = true;

  // Set values in case steps are omitted
  eRs =   uSave.lastState.seriesR;          eLs =  uSave.lastState.seriesL;
  eR50 =  uSave.lastState.valueRRef[R50];   eR5K = uSave.lastState.valueRRef[R5K];
  e1Meg = uSave.lastState.resInput;         eCin = uSave.lastState.capInput;

  arg = SCmd.next();
  if (arg == NULL || atoi(arg) == 0)
    {
    Serial.println("Tuneup requires steps 1 to 4, in order, omissions allowed:");
    Serial.println("   1-Connect a solid short and type  TUNEUP 1");
    Serial.println("   2-Connect known value R=50 and type  TUNEUP 2 R (the true value of R)");
    Serial.println("   3-Connect known value R=5K and type  TUNEUP 3 R (the true value of R)");
    Serial.println("   4-Leave measurement terminals open and type  TUNEUP 4");
    Serial.println("   5-Type either  TUNEUP 5  to enter tuneup values to corrections, or");
    Serial.println("   6-Type  PARAM1 99  to restore default values");
    Serial.println("Notes: All values can later be changed, individually, with PARAM1 and PARAM2 commands.");
    Serial.println("       Be sure to run TUNEUP 5 or TUNEUP 6 any of  1, 2, 3 or 4.");
    return;
    }

  else if (atoi(arg) == 1)    // Calibrate Rs (500 Hz) and Ls (40 KHz)
    {
    doTuneup = true;
    Serial.println("Short should be connected across Z terminals.  Repeat TUNEUP 1 if not shorted now.");
    Serial.println("Doing short circuit tests...CAL 500 Hz...WAIT...");
    // verboseData = true;
    // We assume the R50 value is close to 50.00 Ohms
    uSave.lastState.iRefR = R50;           // Use 50 Ohm ref
    // Now measure  a short with input series corrections turned off
    uSave.lastState.seriesR = 0.0;         // Remove any corrections
    uSave.lastState.seriesL = 0.0;
    // First, a high accuracy CAL to compare meas and ref channels
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    nFreq = 6;                  // 500 Hz
    setSwitch(IMPEDANCE_38);
    setUpNewFreq(nFreq);
    DC1.amplitude(dacLevel);     // Turn on sine wave
    delay(100);
    meas3=0.0;  meas4=0.0;
    for(ju=1; ju<=100; ju++)
      {
      CalCommand();
      meas3 += FreqData[nFreq].vRatio;
      meas4 += FreqData[nFreq].dPhase;
      }
    // Put the ave values into the corrections for 500 Hz:
    FreqData[nFreq].vRatio  = 0.01*meas3;
    FreqData[nFreq].dPhase = 0.01*meas4;
    Serial.print("500 Hz Measurement/Reference Voltage gain = ");
    Serial.println(FreqData[nFreq].vRatio, 7);
    Serial.print("500 Hz Measurement - Reference Phase (deg) = ");
    Serial.println(FreqData[nFreq].dPhase, 3);

    Serial.println("Doing short circuit test...MEASURE 500 Hz...WAIT...");
    setRefR(R50);           // Set 50 Ohm relay
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    setSwitch(IMPEDANCE_38);
    delay(50);
    meas1=0.0; meas2=0.0;
    for(ju=1; ju<=100; ju++)
      {
      measureZ(nFreq);
      checkOverload();
      meas1 += Z[6].real();
      meas2 += Z[6].imag();
      if(verboseData)
        {
        Serial.print("Measured series R = ");
        Serial.println(Z[6].real(), 6);
        }
      delay(2);
      }
    eRs = 0.01 * meas1;
    // Put eRs in as the Rs correction to use
    uSave.lastState.seriesR=eRs;
    Serial.print("500 Hz  Rs = "); Serial.print(eRs, 6); Serial.println("   <----");
    if(verboseData)
      {
      Serial.print("        Xs = ");  Serial.println(0.01*meas2, 6);
      Serial.print("        Ls = ");  Serial.print(1000000000.0*0.01*meas2/3141.593, 1);
      Serial.println(" nH");
      }

    //Do the same at 40 KHz for Ls
    Serial.println("Doing short circuit tests...CAL 40 KHz...WAIT...");
    // We assume the R50 value is close to 50.00 Ohms
    uSave.lastState.iRefR = R50;           // Use 50 Ohm ref
    // Now measure  a short with input series L correction turned off
    uSave.lastState.seriesL = 0.0;
    // First, a high accuracy CAL to compare meas and ref channels
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    nFreq = 13;                  // 40 KHz
    setSwitch(IMPEDANCE_38);
    setUpNewFreq(nFreq);
    DC1.amplitude(dacLevel);     // Turn on sine wave
    delay(100);
    meas3=0.0;  meas4=0.0;
    for(ju=1; ju<=100; ju++)
      {
      CalCommand();
      meas3 += FreqData[nFreq].vRatio;
      meas4 += FreqData[nFreq].dPhase;
      }
    // Put the ave values into the corrections for 40 KHz:
    FreqData[nFreq].vRatio  = 0.01*meas3;
    FreqData[nFreq].dPhase = 0.01*meas4;
    Serial.print("40 KHz Measurement/Reference Voltage gain = ");
    Serial.println(FreqData[nFreq].vRatio, 7);
    Serial.print("40 KHz Measurement - Reference Phase (deg) = ");
    Serial.println(FreqData[nFreq].dPhase, 3);

    Serial.println("Doing short circuit test...MEASURE 40KHz...WAIT...");
    setRefR(R50);           // Set 50 Ohm relay
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    setSwitch(IMPEDANCE_38);
    delay(50);
    meas1=0.0; meas2=0.0;
    for(ju=1; ju<=100; ju++)
      {
      measureZ(nFreq);
      checkOverload();
      meas1 += Z[13].real();
      meas2 += Z[13].imag();
      if(verboseData)
        {
        Serial.print("Measured series X = ");
        Serial.println(Z[13].imag(), 6);
        }
      delay(2);
      }
    eLs = 0.01 * meas2;
    // Put eLs in as the Ls correction to use
    uSave.lastState.seriesL=eLs/251327.4;       // X/(2*pi*40KHz) in Hy
    if(verboseData)
      {
      Serial.print("40 KHz Rs="); Serial.print(0.01*meas1, 6);
      Serial.print(",  Xs=");  Serial.print(eLs, 6);
      Serial.print(",  Ls=");  Serial.print(1000000000.0*eLs/251327.4, 1);
      }
    else
      {
      Serial.print("40 KHz  Ls = "); Serial.print(1000000000.0*eLs/251327.4, 1);
      }
    Serial.println(" nH   <----");
    doTuneup = false;
    Serial.println("TUNEUP 1 Short Circuit tests COMPLETE");
    Serial.println("Ready for known 50 Ohm test, TUNEUP 2 R50");
    Serial.println();
    }

  // "TUNEUP 2 R" - Measure a known resistor, about 50 Ohms, to correct for minor error in ref 50 Ohm
  else if (atoi(arg) == 2)
    {
    arg = SCmd.next();       // This is the precision resistor value
    if(arg != NULL)          // Is there an arg?
      {
      rPrecise = (float)atof(arg);
      Serial.print("rPrecise = ");  Serial.println(rPrecise, 3);
      if (rPrecise<45.0 || rPrecise>55.0)
        {
        Serial.println("WARNING - Precision resistor should be within 10% of 50 Ohms");
        }
      doTuneup = true;
      Serial.println("Measuring \"50 Ohm\" precision resistor...CAL 500 Hz...WAIT...");
      // First, a high accuracy CAL to compare meas and ref channels
      uSave.lastState.ZorT = IMPEDANCE;
      uSave.lastState.iRefR = R50;
      uSave.lastState.SingleorSweep = SINGLE;
      nFreq = 6;                  // 500 Hz
      setSwitch(IMPEDANCE_38);
      setUpNewFreq(nFreq);
      DC1.amplitude(dacLevel);     // Turn on sine wave
      delay(100);
      meas3=0.0;  meas4=0.0;
      for(ju=1; ju<=50; ju++)
        {
        CalCommand();
        meas3 += FreqData[nFreq].vRatio;
        meas4 += FreqData[nFreq].dPhase;
        }
      // Put the ave values into the corrections for 500 Hz:
      FreqData[nFreq].vRatio  = 0.02*meas3;
      FreqData[nFreq].dPhase = 0.02*meas4;
      Serial.print("500 Hz Measurement/Reference Voltage gain = ");
      Serial.println(FreqData[nFreq].vRatio, 7);
      Serial.print("500 Hz Measurement - Reference Phase (deg) = ");
      Serial.println(FreqData[nFreq].dPhase, 3);

      Serial.println("Measuring precision 50-Ohm resistor...MEASURE 500 Hz...WAIT...");
      setRefR(R50);           // Set 50 Ohm relay
      uSave.lastState.ZorT = IMPEDANCE;
      uSave.lastState.SingleorSweep = SINGLE;
      setSwitch(IMPEDANCE_38);
      delay(50);
      meas1=0.0; meas2=0.0;
      for(ju=1; ju<=50; ju++)
        {
        measureZ(nFreq);
        checkOverload();
        meas1 += Z[6].real();    // Z[] is fully corrected for strays and for input circuitry
        meas2 += Z[6].imag();
        delay(2);
        }
     if(verboseData)
        {
        Serial.print("Measured R, using R50 value of "); Serial.print(uSave.lastState.valueRRef[R50], 3);
        Serial.print(" = ");  Serial.println(0.02*meas1, 3);
        }
      eR50 =  (rPrecise / (0.02*meas1)) * uSave.lastState.valueRRef[R50];
      uSave.lastState.valueRRef[R50] = eR50;            // For use by other TUNEUP steps
      Serial.print("500 Hz: Measured internal 50 Ohm Ref Res = ");
      Serial.print(eR50, 3); Serial.println(" <---");
      }
    else
      {
      Serial.println("TUNEUP 2 requires the test resistor value, like  TUNEUP 3 50.13");
      return;
      }
    Serial.println("TUNEUP 2, Precise 50-Ohm, tests COMPLETE");
    Serial.println("Ready for known 5K Ohm test, TUNEUP 3 R5K");
    Serial.println();
    }

  // "TUNEUP 3 R" - Measure a known resistor, ~5000 Ohms, to correct for minor error in ref 5000 Ohm
  else if (atoi(arg) == 3)
    {
    arg = SCmd.next();       // This is the precision resistor value
    if(arg != NULL)          // Is there an arg?
      {
      rPrecise = (float)atof(arg);
      Serial.print("rPrecise = ");  Serial.println(rPrecise, 2);
      if (rPrecise<4500.0 || rPrecise>5500.0)
        {
        Serial.println("WARNING - Precision resistor should be within 10% of 5000 Ohms");
        }
      doTuneup = true;
      Serial.println("Measuring \"5000 Ohm\" precision resistor...CAL 500 Hz...WAIT...");
      // First, a high accuracy CAL to compare meas and ref channels
      uSave.lastState.ZorT = IMPEDANCE;
      uSave.lastState.iRefR = R5K;
      uSave.lastState.SingleorSweep = SINGLE;
      nFreq = 6;                  // 500 Hz
      setSwitch(IMPEDANCE_38);
      setUpNewFreq(nFreq);
      DC1.amplitude(dacLevel);     // Turn on sine wave
      delay(100);
      meas3=0.0;  meas4=0.0;
      for(ju=1; ju<=50; ju++)
        {
        CalCommand();
        meas3 += FreqData[nFreq].vRatio;
        meas4 += FreqData[nFreq].dPhase;
        }
      // Put the ave values into the corrections for 500 Hz:
      FreqData[nFreq].vRatio  = 0.02*meas3;
      FreqData[nFreq].dPhase = 0.02*meas4;
      Serial.print("500 Hz Measurement/Reference Voltage gain = ");
      Serial.println(FreqData[nFreq].vRatio, 7);
      Serial.print("500 Hz Measurement - Reference Phase (deg) = ");
      Serial.println(FreqData[nFreq].dPhase, 3);

      Serial.println("Measuring precision 5000-Ohm resistor...MEASURE 500 Hz...WAIT...");
      setRefR(R5K);           // Set 5000 Ohm relay
      uSave.lastState.ZorT = IMPEDANCE;
      uSave.lastState.SingleorSweep = SINGLE;
      setSwitch(IMPEDANCE_38);
      delay(50);
      meas1=0.0; meas2=0.0;
      for(ju=1; ju<=50; ju++)
        {
        measureZ(nFreq);
        checkOverload();
        meas1 += Z[6].real();
        meas2 += Z[6].imag();
        if(verboseData)
          {
          Serial.print("Measured R = ");
          Serial.println(Z[6].real(), 3);
          }
        delay(2);
        }
     if(verboseData)
        {
        Serial.print("Measured R, using R5K value of "); Serial.print(uSave.lastState.valueRRef[R5K], 3);
        Serial.print(" = ");  Serial.println(0.02*meas1, 3);
        }
      eR5K =  (rPrecise / (0.02*meas1)) * uSave.lastState.valueRRef[R5K];
      uSave.lastState.valueRRef[R5K] = eR5K;   // For use by other stages of UPDATE
      Serial.print("500 Hz: Measured internal 5 KOhm Ref Res = ");
      Serial.print(eR5K, 1); Serial.println(" <---");
      }
    else      // Missing resistor value
      {
      Serial.println("TUNEUP 3 requires the test resistor value, like  TUNEUP 3 5002.3");
      return;
      }
    doTuneup = false;
    Serial.println("TUNEUP 3, Precise 5K-Ohm, tests COMPLETE");
    Serial.println("Ready for Open Circuit tests, TUNEUP 4");
    Serial.println();
    }

  // TUNEUP 4  - Measure 1 Meg input at 500 Hz, Input capacity at 40 KHz
  else if (atoi(arg) == 4)
    {
    doTuneup = true;
    Serial.println("Measuring Open Circuit (1 Meg)...CAL 500 Hz...WAIT...");
    // First, a high accuracy CAL to compare meas and ref channels
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.iRefR = R5K;
    uSave.lastState.SingleorSweep = SINGLE;
    nFreq = 6;                  // 500 Hz
    setSwitch(IMPEDANCE_38);
    setUpNewFreq(nFreq);
    DC1.amplitude(dacLevel);     // Turn on sine wave
    delay(100);
    meas3=0.0;  meas4=0.0;
    for(ju=1; ju<=50; ju++)
      {
      CalCommand();
      meas3 += FreqData[nFreq].vRatio;
      meas4 += FreqData[nFreq].dPhase;
      }
    // Put the ave values into the corrections for 500 Hz:
    FreqData[nFreq].vRatio  = 0.02*meas3;
    FreqData[nFreq].dPhase = 0.02*meas4;
    Serial.print("500 Hz Measurement/Reference Voltage gain = ");
    Serial.println(FreqData[nFreq].vRatio, 7);
    Serial.print("500 Hz Measurement - Reference Phase (deg) = ");
    Serial.println(FreqData[nFreq].dPhase, 3);

    Serial.println("Measuring Open Circuit (1 Meg and Cin)...MEASURE 500 Hz...WAIT...");
    setRefR(R5K);           // Set 5000 Ohm relay
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    setSwitch(IMPEDANCE_38);
    delay(50);
    meas1=0.0; meas2=0.0;
    for(ju=1; ju<=50; ju++)
      {
      measureZ(nFreq);
      checkOverload();
      // The input Z, corrected for the 0.22 uF is in complex variable Ztuneup
      meas1 += (Cone / Ztuneup).real();       // Sum G for (G+jB) = 1/(R+jX)
      meas2 += (Cone / Ztuneup).imag();       // Sum B
      delay(2);
      }
    meas1 *= 0.02;                 // Get average G
    meas2 *= 0.02;                 //   and B
    e1Meg = 1.0 / meas1;
    uSave.lastState.resInput = e1Meg;
    Serial.print("At 500 Hz, Measured R1Meg, using R5K value of ");
    Serial.print(uSave.lastState.valueRRef[R5K], 3);
    Serial.print(" = ");  Serial.print(e1Meg, 1); Serial.println(" <---");

    Serial.println("Measuring Open Circuit (Cin)...CAL 40 KHz...WAIT...");
    // First, a high accuracy CAL to compare meas and ref channels
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.iRefR = R5K;
    uSave.lastState.SingleorSweep = SINGLE;
    nFreq = 13;                  // 40 KHz
    setSwitch(IMPEDANCE_38);
    setUpNewFreq(nFreq);
    DC1.amplitude(dacLevel);     // Turn on sine wave
    delay(100);
    meas3=0.0;  meas4=0.0;
    for(ju=1; ju<=50; ju++)
      {
      CalCommand();
      meas3 += FreqData[nFreq].vRatio;
      meas4 += FreqData[nFreq].dPhase;
      }
    // Put the ave values into the corrections for 40 KHz:
    FreqData[nFreq].vRatio  = 0.02*meas3;
    FreqData[nFreq].dPhase = 0.02*meas4;
    Serial.print("40 KHz Measurement/Reference Voltage gain = ");
    Serial.println(FreqData[nFreq].vRatio, 7);
    Serial.print("40 KHz Measurement - Reference Phase (deg) = ");
    Serial.println(FreqData[nFreq].dPhase, 3);

    Serial.println("Measuring Open Circuit (Cin)...MEASURE 40 KHz...WAIT...");
    setRefR(R5K);           // Set 5000 Ohm relay
    uSave.lastState.ZorT = IMPEDANCE;
    uSave.lastState.SingleorSweep = SINGLE;
    setSwitch(IMPEDANCE_38);
    delay(50);
    meas1=0.0; meas2=0.0;
    for(ju=1; ju<=50; ju++)
      {
      measureZ(nFreq);
      checkOverload();
      // The input Z, corrected for the 0.22 uF is in complex variable Ztuneup
      // Since a sizeable part of the input capacity is outboard of the
      // 0.22 uF cap, we will use the un-adjusted version, Ztuneup0:
      meas1 += (Cone / Ztuneup0).real();       // Sum G for (G+jB) = 1/(R+jX)
      meas2 += (Cone / Ztuneup0).imag();       // Sum B
      delay(2);
      }
    meas1 *= 0.02;                 // Get average G
    meas2 *= 0.02;                 //   and B
    eCin = meas2 / (251327.4);
    uSave.lastState.capInput = eCin;
    Serial.print("At 40 KHz, Measured Cin, using R5K value of ");
    Serial.print(uSave.lastState.valueRRef[R5K], 3);
    Serial.print(", = ");  Serial.print(1.0E12*eCin, 1); Serial.println(" pF  <---");
    Serial.print("     Before TUNEUP, value was: "); Serial.println(1.0E12*eCinSave, 1);
    Serial.println("TUNEUP 4, Open Circuit, tests complete");
    Serial.println("------------------------------------------------------------------");
    Serial.println("Summary, the other 5 values:");
    Serial.print("   Rs = ");Serial.print(eRs, 4);
    Serial.print("     Before TUNEUP, value was: "); Serial.println(eRsSave, 4);
    Serial.print("   Ls = ");Serial.print(1.0E9*eLs, 1);
    Serial.print("     Before TUNEUP, value was: "); Serial.println(1.0E9*eLsSave, 1);
    Serial.print("   R50 = ");Serial.print(eR50, 3);
    Serial.print("     Before TUNEUP, value was: "); Serial.println(eR50Save, 3);
    Serial.print("   R5K = ");Serial.print(eR5K, 1);
    Serial.print("     Before TUNEUP, value was: "); Serial.println(eR5KSave, 1);
    Serial.print("   R1Meg = ");Serial.print(e1Meg, 0);
    Serial.print("     Before TUNEUP, value was: "); Serial.println(e1MegSave, 0);
    Serial.println("Ready for Save or Cancel, TUNEUP 5 or TUNEUP 6");
    Serial.println();
    }

  else if (atoi(arg) == 5)
    {
    Serial.println("Making measurements permanent. Use PARAM1 or PARAM2 to alter these.");
    uSave.lastState.seriesR=eRs;          uSave.lastState.seriesL=eLs;
    uSave.lastState.valueRRef[R50]=eR50;  uSave.lastState.valueRRef[R5K]=eR5K;
    uSave.lastState.resInput=e1Meg;       uSave.lastState.capInput=eCin;
    saveStateEEPROM();
    }

  else if (atoi(arg) == 6)
    {
    Serial.println("Restoring current parameters (not defaults).  Use PARAM1 or PARAM2 to alter these.");
    uSave.lastState.seriesR=eRsSave;          uSave.lastState.seriesL=eLsSave;
    uSave.lastState.valueRRef[R50]=eR50Save;  uSave.lastState.valueRRef[R5K]=eR5KSave;
    uSave.lastState.resInput=e1MegSave;       uSave.lastState.capInput=eCinSave;
    saveStateEEPROM();
    }
  }

// LINLOG rs ts rd td   can change the units used for outputs to the serial monitor,
// or to the Touch Display.  The four numbers following the LINLOG command set:
//    rs=0  Reflection coefficient to Serial in dB, and phase in degrees
//    rs=1  Reflection coefficient to Serial in magnitude (0,1) and phase in degrees
//    rs=2  Reflection data to Serial as equivalent Series Impedance or Parallel Suseptance
//          (see SERPAR command)
//    ts=0  Transmission data to Serial in dB, and phase in degrees
//    ts=1  Transmission dada to Serial in magnitude and phase in degrees
//    rd=0  Reflection coefficient to Touch Display in dB, and phase in degrees
//    rd=1  Reflection coefficient to Touch Display in magnitude (0,1) and phase in degrees
//    rd=2  Reflection data to Touch Display as equivalent Series Impedance and Parallel Suseptance
//    td=0  Transmission data to Touch Display in dB, and phase in degrees
//    td=1  Transmission dada to Touch Display in magnitude and phase in degrees
// Default is the original values for backward compatibility: "LINLOG 2 1 2 0"
// Short commands work if you do not want to change the Touch Display.  For nstance, "LINLOG 0 0" will
// just make the serial output in dB and degrees for both reflection coefficient and transmission.
// The command "LINLOG" without any parameters will return the currrent settings, such as "LINLOG 0 0 1 1".
// Settings are saved in EEPROM and so survive the power shutdown.
// Rev 0.5.9
void LinLogCommand(void)
  {
  char *arg;
  char ch[35];

  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.rsData = (uint8_t)atoi(arg);
  else
     {
     sprintf(ch, "Currently:  LINLOG %d %d %d %d",
       (int)uSave.lastState.rsData,
       (int)uSave.lastState.tsData,
       (int)uSave.lastState.rdData,
       (int)uSave.lastState.tdData);
     Serial.println(ch);
     return;
     }
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.tsData = (uint8_t)atoi(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.rdData = (uint8_t)atoi(arg);
  arg = SCmd.next();
  if (arg != NULL)
     uSave.lastState.tdData = (uint8_t)atoi(arg);
  saveStateEEPROM();
  }

// Command to change baud rate -
// In Teensy 3.6 this does nothing.  It is always 12 MBit/sec
void BaudCommand(void)
  {
  /*     Rev 0.5.9  */
  }

// "DUMP_E" command Serial prints the contents of the EEPROM, byte by byte.
void DumpECommand(void)
  {
  uint16_t i;
  Serial.print("Dump of ");
  Serial.print(sizeof(saveState));
  Serial.println(" bytes of EEPROM in decimal:");
  for (i = 0; i < sizeof(saveState); i++)
     {
      Serial.print(i);
      Serial.print(",");
      Serial.println(EEPROM.read(i));
     }
  }

// "WRITE_E i val" command.
void WriteECommand()
  {
  char *arg;
  uint16_t i;      // EEPROM location
  uint8_t val;     // Byte to write

  arg = SCmd.next();
  if (arg != NULL)
     i = (uint16_t)atoi(arg);
  else
     {
     Serial.println("Error: Command WRITE_E needs two parameters");
     return;
     }

  arg = SCmd.next();
  if (arg != NULL)
     val = (uint8_t)atoi(arg);
 else
     {
     Serial.println("Error: Command WRITE_E needs two parameters");
     return;
     }
  EEPROM.write(i, val);
  }

// Command for annotating printout; Otherwise, CSV
void AnnotateCommand()
  {
  char *arg;
  arg = SCmd.next();
  if (arg == NULL)
    return;
  else if (strcmp(arg, "0") == 0)
    annotate = false;
  else if (strcmp(arg, "1") == 0)
    {
    annotate = true;
    Serial.println("Annotation enabled");
    }
  }

// ==============  Commands to support the nanoVNA interface  ===========

// calCommand, for "cal", is different than regular serial Cal command for "CAL"
void calCommand()
  {
    int iii;
    for (iii=1;  iii<=13; iii++)
        {
        Serial.print(FreqData[iii].vRatio,8);
        Serial.print(",");
        Serial.print(FreqData[iii].dPhase,5);
        Serial.print(",");
        Serial.print(FreqData[iii].thruRefAmpl,8);
        Serial.print(",");
        Serial.println(FreqData[iii].thruRefPhase,5);
        }
     sendEOT();
  }

/* Commands for compatibility with the nanoVNA.  This allows software made
 *  for the nanoVNA to control the AVNA.  These commands must be in lower
 *  case where all the AVNA commands must be in caps.  These respond to either
 *  the Serial (USB) port or to harware (RS-232) type serial connected to the
 *  Teensy serial 4.
 */
void infoCommand()
  {
  // To make nanovna-saver happy, we will need no extra data:
  doingNano = true;
  annotate = false;
  verboseData = false;
  if (portSelect & NANO_USE_USB)
    {
    Serial.println("\r\nAVNA-1\r");
    Serial.println("\r\nNanoVNA-H\r");   //  imitating nanoVNA, to be removed soon
    Serial.println("Board: AVNA + Teensy3.6");
    Serial.print("ch> "); Serial.send_now();
    }
  if (portSelect & NANO_USE_HW4)
    {
    HWSERIAL4.println("\r\nAVNA-1\r");
    HWSERIAL4.println("\r\nNanoVNA-H\r");
    HWSERIAL4.println("Board: AVNA + Teensy3.6");
    HWSERIAL4.print("ch> ");
    }
  }

void helpCommand()
  {
  if (portSelect & NANO_USE_USB)
    {
    Serial.println("\r\nHelp: N/A");
    Serial.print("ch> "); Serial.send_now();
    }
  if (portSelect & NANO_USE_HW4)
    {
    HWSERIAL4.println("\r\nHelp: N/A");
    HWSERIAL4.print("ch> ");
    }
  }
void versionCommand()
  {
  doingNano = true;
  if (portSelect & NANO_USE_USB)
    {
    Serial.print("v0.");
    Serial.print(CURRENT_VERSION);
    Serial.println(".0-0-avna");
    Serial.print("ch> "); Serial.send_now();
    }
  if (portSelect & NANO_USE_HW4)
    {
    HWSERIAL4.print("\r\nv0.");
    HWSERIAL4.print(CURRENT_VERSION);
    HWSERIAL4.println(".0-0-avna");
    HWSERIAL4.print("ch> ");
    }
  }

void captureCommand()
  {
  sendEOT();
  }

void frequenciesCommand()
  {
  uint16_t kk;
  doingNano = true;
  if (portSelect & NANO_USE_USB)
    {
    for (kk=0; kk<sweepPoints; kk++)
        Serial.println((uint16_t)dataFreq[kk]);
    Serial.print("ch> "); Serial.send_now();
    }
  if (portSelect & NANO_USE_HW4)
    {
    for (kk=0; kk<sweepPoints; kk++)
        HWSERIAL4.println((uint16_t)dataFreq[kk]);
    HWSERIAL4.print("ch> ");
    }
  }

// data command selects S11 or S21 data to send.
// This sets it up, and loop() sends it line by line.
void dataCommand()
  {
  char *arg;
  doingNano = true;
  arg = SCmd.next();
  if (arg != NULL)
     {
     nanoState = DATA_READY_NANO;
     // sendDataType  0=sending S11 data
     //               1=sending S21 data
     sendDataType = (uint16_t)atoi(arg);
     sendDataCount = 0;  // Start sending
     }
  else      // No data type was specified
     {
     sendDataCount = 9999;  // Something wrong, no sending
     }
  // Hold up commands (store in serialInBuffer) until data is sent by serial
  commandOpen = false;
  }

// Just send a line of data, to be called by loop()
void sendDataLine(uint16_t nSD)
  {
  if (portSelect & NANO_USE_USB)
    {
    if(sendDataType == 0)
      {
      // These are reflection coefficient (S11) in re and im form
      Serial.print  (dataReReflec[nSD],6);
      Serial.print  (" ");
      Serial.println(dataImReflec[nSD],6);
      }
    else
      {
      Serial.print  (dataReTrans[nSD],6);
      Serial.print  (" ");
      Serial.println(dataImTrans[nSD],6);
      }
    }

  if (portSelect & NANO_USE_HW4)
    {
    if(sendDataType == 0)
      {
      HWSERIAL4.print  (dataReReflec[nSD],6);
      HWSERIAL4.print  (" ");
      HWSERIAL4.println(dataImReflec[nSD],6);
      }
    else
      {
      HWSERIAL4.print  (dataReTrans[nSD],6);
      HWSERIAL4.print  (" ");
      HWSERIAL4.println(dataImTrans[nSD],6);
      }
    }
  }

/* sweep–sets the start/stop for the next sweep.
 * command stops the automatic sweep (per QRP)-Usage: sweep {start(Hz)} [stop]
 * [points]-If no inputs,then sweeps current setup.
 * For the nanovna this sets things up, but  does not trigger a sweep.
 * Here it triggers the series of measurements.  Also here, it is not
 * restricted to 101 points but anything from 2 to 1601.
 * Data measurements occur one at a time in loop() to not hold things up.
 */
void sweepCommand()
  {
  char *arg;
  uint16_t kk;

  arg = SCmd.next();
  if (arg != NULL)       // There are arguments
     {
     sweepStart = (uint16_t)atoi(arg);
     // nanoState stays until end of printing in loop()
     nanoState = MEASURE_NANO;
     }
  else                   // Just send back current start, stop, points
     {
     if (portSelect & NANO_USE_USB)
       {
       Serial.print(sweepStart);
       Serial.println(" ");
       Serial.print(sweepStop);
       Serial.println(" ");
       Serial.print(sweepPoints);
       }
     if (portSelect & NANO_USE_HW4)
       {
       HWSERIAL4.print(sweepStart);
       HWSERIAL4.println(" ");
       HWSERIAL4.print(sweepStop);
       HWSERIAL4.println(" ");
       HWSERIAL4.print(sweepPoints);
       }
     sendEOT();
     return;
     }
  // We've already setup start, and need to do two more.
  arg = SCmd.next();
  if (arg != NULL)
     sweepStop = (uint16_t)atoi(arg);
  arg = SCmd.next();
  if (arg != NULL)
     sweepPoints = (uint16_t)atoi(arg);
  for(kk=0; kk<sweepPoints; kk++)
     dataFreq[kk] = sweepStart + (float)kk * ((sweepStop - sweepStart) / ((float)sweepPoints - 1.0));
  // For now, nano emulate is always 50 Ohms
  uSave.lastState.iRefR = R50;
  setRefR(R50);
  // We are ready to do sweep points from loop() and so indicate:
  sweepCurrentPoint = 0;
  // For use by data command, to know how many to send
  totalDataPoints = sweepPoints;
  // Hold up commands(store in serialInBuffer) until data is collected
  commandOpen = false;
  // For nanoVNA style sweep, all data is collected at nFreq=0.  The 13 sweep
  // points for the serial SWEEP are not used.  Thus:
  nFreq = 0;
  DC1.amplitude(dacLevel);     // Turn on sine wave
  }

// To support the sweep command, we need to get reflection and transmission data
// points for one frequency of index nf:
void getDataPt(uint16_t mf)
  {
  //Complex Crefl(0.0, 0.0);

  // First a reflection measurement
  uSave.lastState.ZorT = IMPEDANCE;
  setSwitch(IMPEDANCE_38);           // Connect for Z measure
  FreqData[0].freqHz = dataFreq[mf];
  prepMeasure(FreqData[0].freqHz);   // This sets FreqData[0]
  setUpNewFreq(0);   // Sets up sample frequency and waveform freq
  delay(ZDELAY);        // Delay until level is constant
  measureZ(0);      //  uses FreqData[0], result is Z[0]

 // Crefl = (Z[0] - Complex(50.0, 0.0)) / (Z[0] + Complex(50.0, 0.0));
 // dataReReflec[mf] = Z[0].real();  //Crefl.real();
 // dataImReflec[mf] = Z[0].imag();  //Crefl.imag();
  Complex Zo(uSave.lastState.valueRRef[uSave.lastState.iRefR], 0.0);
  ReflCoeff = (Z[0] - Zo) / (Z[0] + Zo);
  dataReReflec[mf] = ReflCoeff.real();
  dataImReflec[mf] = ReflCoeff.imag();

  // And also a transmission measurement
  //  DC1.amplitude(dacLevel);     // Turn on sine wave
  //  setUpNewFreq(nFreq);
  // Delay until level is constant
  uSave.lastState.ZorT = TRANSMISSION;
  setSwitch(TRANSMISSION_37);
  measureT();      // result is Tmeas
  dataReTrans[mf] = Tmeas.real();
  dataImTrans[mf] = Tmeas.imag();
  }

// For nanoVNA this starts sweep back up.  AVNA doesn't need that
// so just acknowledge.
void resumeCommand()
  {
  sendEOT();
  }

// Send end of transmission to nanoVNA types
void sendEOT()
  {
  if (portSelect & NANO_USE_USB)
      {
      Serial.print("ch> ");
      Serial.send_now();
      }
  if (portSelect & NANO_USE_HW4)
      HWSERIAL4.print("ch> ");
  }

// This is the default Command handler
void unrecognized()
  {
  if (portSelect & NANO_USE_USB)
      Serial.println("What cmd?");
  if (portSelect & NANO_USE_HW4)
      {
      HWSERIAL4.println("?");
      HWSERIAL4.print("ch> ");
      }
  }
