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

// ------------------- Touch LCD Menu Response Functions  --------------------------

// Menus at bottom occupy vertically 200 to 240. Lines at top are 0 to 38.
void tToInstrumentHome(void)
  {
  tft.fillRect(0, 0, 320, 200, ILI9341_BLACK);
  topLine1();  topLine2();
  instrument = ALL_IDLE;
  avnaState = 0;
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print("                    Welcome!");
  tft.setCursor(0, 60);
  tft.setFont(Arial_10);
  tft.print("  Select from the bottom menu items.");
  tft.setFont(Arial_12);
  tft.setCursor(0, 80);
  tft.print("    1-Audio Vector Network Analyzer");
  tft.setCursor(0, 100);
  tft.print("    2-Vector Voltmeter");
  tft.setCursor(0, 120);
  tft.print("    3-Audio Spectrum Analyzer");
  tft.setCursor(0, 140);
  tft.print("    4-Signal Generators");
  tft.setCursor(0, 160);
  tft.print("    5-Service & Calibration");
  dataValidTSweep = false;
  dataValidZSweep = false;
  clearStatus();
  }

void tService(void)
  {
  instrument = ALL_IDLE;
  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);
  tft.fillRect(0, 38, tft.width(), 160, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(0, 38);
  tft.print("      Service Menu");;
  tft.setCursor(0, 60);
  tft.setFont(Arial_10);
  tft.print("  Select from bottom row");
  }

void tTouchCal(void)
  {
  unsigned long tm = millis();
  uint16_t TxMin, TxMax, TyMin, TyMax;

  TxMin = 5000;  TxMax = 0;
  TyMin = 5000;  TyMax = 0;

  instrument = TOUCH_CAL;
  topLine1();
  drawScreenSaveBox(ILI9341_BLACK);  // Remove screen save
  tft.fillRect(0, 38, 320, 202, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(0, 38);
  tft.print("      Touch Screen Calibration");
  tft.setCursor(0, 60);
  tft.setFont(Arial_10);
  tft.print(" 1-Multiple Touches of upper-left corner");
  tft.setCursor(0, 80);
  tft.print(" 2-Multiple touches of lower-right corner");
  tft.setCursor(0, 100);
  tft.print(" 3-Touch upper-right corner to Exit");
  while(1)
    {
    //   boolean istouched = ts.touched();
    if (ts.touched()  &&  (millis()-tm) > 150)           // Set repeat rate and de-bounce or T_REPEAT
      {
      tm = millis();
      TS_Point p = ts.getPoint();
      if(p.x<1500 && p.x<TxMin) TxMin = p.x;
      if(p.y<1500 && p.y<TyMin) TyMin = p.y;
      if(p.x>=1500 && p.x>TxMax) TxMax = p.x;
      if(p.y>=1500 && p.y>TyMax) TyMax = p.y;
      //Serial.print("X = "); Serial.print(p.x);
      //Serial.print("\tY = "); Serial.print(p.y);
      //Serial.print("\tPressure = "); Serial.println(p.z);
      tft.fillRect(0, 120, 320, 120, ILI9341_BLACK);
      tft.setCursor(0, 120);
      tft.print(" X =");
      tft.setCursor(100, 120);
      tft.print(p.x);
      tft.setCursor(0, 140);
      tft.print(" Y =");
      tft.setCursor(100, 140);
      tft.print(p.y);

      tft.setCursor(0, 160);
      tft.print(" New Corner Values:");
      tft.setFont(Arial_12);
      tft.setCursor(20, 180);
      tft.print(TxMin);
      tft.setCursor(80, 180);
      tft.print(TyMin);
      tft.setCursor(120, 200);
      tft.print(TxMax);
      tft.setCursor(180, 200);
      tft.print(TyMax);

      if (p.x>2000 && p.y<1500)
        {
        // Check validity and install new limits
        if(TxMin>100 && TxMin<1000 && TyMin>100 && TyMin<1000 &&
           TxMax>3000 && TxMax<4500 && TyMax>3000 && TyMax<4500)
           {
           uSave.lastState.xMin=TxMin;  uSave.lastState.xMax=TxMax;
           uSave.lastState.yMin=TyMin;  uSave.lastState.yMax=TyMax;
           saveStateEEPROM();
           Serial.println("Touch Screen Cal and save to EEPROM complete.");
           }
        else
           {
           tft.setCursor(0, 80);
           tft.print(" INVALID CORNER VALUES - Not saved");
           }
        tft.fillRect(0, 38, tft.width(), 122, ILI9341_BLACK);
        tft.setCursor(0, 130);
        tft.print("     Exiting Touch Cal");
        delay(5000);
        tft.fillRect(0, 38, 320, 202, ILI9341_BLACK);

        tToInstrumentHome();   // Get back to regular stuff.
        currentMenu = 18; menuItem = 0;
        return;
        }
      }
    }
  }

void tVInCal(void)
  {
  float32_t vRMS, vPP, vRatioErr;
  bool missingData = false;
  setI2SFreq(S96K);   // 96 KHz sample rate
  instrument = VIN_CAL;
  if(currentMenu==20 && menuItem==1)    // Measure is requested
    {
    tft.fillRect(0, 187, 320, 12, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_10);
    tft.setCursor(0, 188);
    tft.print("      WAIT      WAIT     WAIT");

    vPP = 0.0f;
    vRMS = 0.0f;
    if(pkDet.available()) pkDet.readPeakToPeak();
    if(rms1.available()) rms1.read();
    Serial.println("INPUT V CAL - Wait 5 sec for measurement.");
    missingData = false;
    for(int ii =0; ii<10; ii++)
      {
      delay(500);
      if(pkDet.available())
        vPP  += pkDet.readPeakToPeak();
      else
        missingData = true;
      if(rms1.available())
        vRMS += rms1.read();
      else
        missingData = true;
      }

    vRMS *= 0.1f;   vPP *= 0.1f;
    if(!missingData)
      {
      uSave.lastState.VVMCalConstant = 0.00000430688/vRMS;
      Serial.print("New VVMCalConstant = ");
      Serial.println(uSave.lastState.VVMCalConstant, 11);
      }
    else
      Serial.println("CAL ERROR - Missing data.  Try again.");
    Serial.print("V p-p = ");  Serial.println( vPP,  7);
    Serial.print("V rms = ");  Serial.println( vRMS, 7);
    vRatioErr = 100.0f*((2.82843f*vRMS/vPP)-1);
    if(fabsf(vRatioErr)>2.0f)
      {
      Serial.print("WARNING - Poor quality sine wave, rms/peak error of ");
      Serial.print(vRatioErr);
      Serial.println("%");
      }

    // Now do a cal of the spectrum analyzer
    // Get started
    pwr10DB = -1000.0f;
    pwr10=0.0f; doFFT(); delay(600);
    pwr10=0.0f; doFFT(); delay(600);
    countAve = 0;  // Global number of doFFT()
    while(pwr10DB < -999.0f)
      {
      pwr10 = 0.0f;
      doFFT();
      }
    Serial.print("For ASA Cal, pwr10DB = ");   Serial.println(pwr10DB, 3);
    // Input power is 0.1^2 / 50 or -6.9897 dBm.  FFT is pwr10DB.  Offset is
    uSave.lastState.SAcalCorrectionDB = -6.9897 - pwr10DB;
    Serial.print("SAcalCorrectionDB = ");   Serial.println(uSave.lastState.SAcalCorrectionDB, 3);
    if(uSave.lastState.SAcalCorrectionDB<2.5f || uSave.lastState.SAcalCorrectionDB>7.5f)
      Serial.println("WARNING - SAcalCorrectionDB normally near 4.5 dB");
    tft.fillRect(0, 187, 320, 12, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_10);
    tft.setCursor(0, 188);
    saveStateEEPROM();
    Serial.println("V-In Cal and save to EEPROM complete.");
    tft.print(" See USB-Serial output for measurement details");
    }
  else  // just write the screen (this includes "Cancel")
    {
    tft.fillRect(0, 38, 320, 202, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_14);
    tft.setCursor(0, 38);
    tft.print("   Intput Voltage Calibration");
    tft.setCursor(0, 70);
    tft.setFont(Arial_10);
    tft.print(" 1-Apply 0.100 V RMS to Transmission Input");
    tft.setCursor(0, 90);
    tft.print("    Sine-Wave Frequency about 1000 Hz");
    tft.setCursor(0, 110);
    tft.print(" 2-Touch \"Measure\" below and wait 5 sec.");
    tft.setCursor(0, 140);
    tft.print(" Hint - 0.100 V RMS = 0.2828 V p-p");
    tft.setCursor(0, 160);
    tft.print(" Hint - Do this cal before Output Voltage cal");
    tft.setCursor(0, 188);
    tft.print(" See USB-Serial output for measurement details");
    mixer2.gain(0, 0.0);   // Turn off signal generatrs
    mixer2.gain(1, 0.0);   // Turn off AVNA source signal
    setSwitch(TRANSMISSION_37);        // Connect as for T measure
    }
  }

void tVOutCal(void)
  {
  double sumV;

  setI2SFreq(S96K);   // 96 KHz sample rate
  instrument = VOUT_CAL;

  setRefR(R50);          // 50 Ohm output
  setSwitch(TRANSMISSION_37);        // Connect for T measure

  // For VVM the AVNA freq source is set to 1030 Hz
  FreqData[0].freqHz = 1030.0f;
  setUpNewFreq(0);
  // The left ('Z') port comes from the fsig gen asg[0].
  asg[0].type = WAVEFORM_SINE;
  asg[0].freq = 1030.0f;
  asg[0].amplitude = 0.14142136f;  //  0.05V RMS, a known value, well below overload
  asg[0].ASGoutputOn = true;
  asg[1].ASGoutputOn = false; asg[2].ASGoutputOn = false; asg[3].ASGoutputOn = false;
  setSigGens();

  // For VVM the AVNA freq source is set to same freq as Sig Gen1
  FreqData[0].freqHz = asg[0].freq;
  // VVM uses the VNA routines and oscillator
  setUpNewFreq(0);

  // Resync the phase of the two sources
  AudioNoInterrupts();
  waveform1.begin(WAVEFORM_SINE);
  waveform1.frequency(factorFreq * asg[0].freq);
  waveform1.amplitude(1.0);
  waveform1.phase(0.0f);
  waveform2.begin(WAVEFORM_SINE);
  waveform2.frequency(factorFreq * asg[0].freq);
  waveform2.amplitude(1.0);
  waveform2.phase(270.0f);
  AudioInterrupts();

  if(currentMenu==21 && menuItem==1)    // Measure is requested
    {
    tft.fillRect(0, 187, 320, 12, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_10);
    tft.setCursor(0, 188);
    tft.print("      WAIT      WAIT     WAIT");
    Serial.println("OUTPUT V CAL - Wait 10 sec for measurement.");
    getFullDataPt();  // Warmup
    sumV = 0.0f;
    for(int ii=0; ii<16; ii++)
      {
      getFullDataPt();
      checkOverload();
      sumV += (double)amplitudeV;
      }
    sumV *= 0.0625000000;  // Account for x16 sum
    Serial.print("Measured VVM RMS Voltage = ");
    Serial.println(uSave.lastState.VVMCalConstant*(float32_t)sumV, 7); // V rms at terminals
    Serial.print("Previous Sig Gen Cal constant = ");
    Serial.println(uSave.lastState.sgCal, 7);
    uSave.lastState.sgCal *= 0.050f/(uSave.lastState.VVMCalConstant*(float32_t)sumV);
    Serial.print("Updated Sig Gen Cal Constant = ");
    Serial.println(uSave.lastState.sgCal, 7);

    if(uSave.lastState.sgCal<1.2f || uSave.lastState.sgCal>2.0f)
      Serial.println("WARNING - Sig Gen Cal constant normally near 1.6");
    tft.fillRect(0, 187, 320, 12, ILI9341_BLACK);
    saveStateEEPROM();
    Serial.println("Vout Cal and save to EEPROM complete.");
    }
  else if(currentMenu==21 && menuItem==5)    // Cancel is requested
    {
    return;
    }
  tft.fillRect(0, 38, 320, 202, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(0, 38);
  tft.print("   Output Voltage Calibration");
  tft.setCursor(0, 60);
  tft.setFont(Arial_10);
  tft.print(" 1-Connect Transmission path.");
  tft.setCursor(0, 80);
  tft.print(" 2-Terminate input with 50 Ohms.");
  tft.setCursor(0, 100);
  tft.print(" 3-Touch \"Measure\" below.");
  tft.setCursor(0, 140);
  tft.print(" Hint - Do this cal after Intput Voltage cal");
  }

void tToAVNAHome(void)
  {
  instrument = AVNA;
  mixer2.gain(0, 0.0);   // Turn off signal generatrs
  mixer2.gain(1, 1.0);   // Turn on AVNA source signal
  topLines();
  avnaState = 0;
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 60);
  tft.print("     AUDIO VETOR NETWORK ANALYZER");
  tft.setCursor(0, 82);
  tft.setFont(Arial_10);
  tft.print("  Select from the bottom menu items.");
  tft.setFont(Arial_12);
  dataValidTSweep = false;
  dataValidZSweep = false;
  clearStatus();
  }

// The signal generator is mutually exclusive with the AVNA instrument but
// always on when the ASA or VVM instruments are on.
void tToASGHome(void)
  {
  setSigGens();

  tft.fillRect(0, 0, tft.width(), 200, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print("      FOUR SIGNAL GENERATORS");
  tft.setCursor(0, 60);
  tft.setFont(Arial_10);
  tft.print(" Adjust Sig Gens with Bottom Menus. Currently");
  tft.setCursor(0, 82);
  tft.setFont(Arial_12);
  for(int jj=0;  jj<3; jj++)
    {
      tft.setCursor(10, 82 + 22*jj);
      tft.print(jj+1);
      tft.setCursor(30, 82 + 22*jj);
      tft.print((uint16_t)asg[jj].freq);
      tft.setCursor(85, 82 + 22*jj);
      tft.print(asg[jj].amplitude, 3);
      tft.setCursor(135, 82 + 22*jj);
      tft.print("V p-p");

      tft.setCursor(210, 82 + 22*jj);
      if(asg[jj].ASGoutputOn)
        tft.print("ON");
      else
        {
        tft.setTextColor(ILI9341_PINK);
        tft.print("OFF");
        }
      tft.setTextColor(ILI9341_YELLOW);
      tft.setCursor(255, 82 + 22*jj);
      if (asg[jj].type == WAVEFORM_SINE)
        tft.print("Sine");
      else if (asg[jj].type == WAVEFORM_SQUARE)
        tft.print("Square");
    }

    tft.setCursor(10, 82 + 66);
    tft.print("4");
    tft.setCursor(30, 82 + 66);
    tft.print(asg[3].freq, 0);       // LPF fco
    tft.setCursor(85, 82 + 66);
    tft.print(asg[3].amplitude, 3);  // SD
    tft.setCursor(135, 82 + 66);
    tft.print("1-sigma");

    tft.setCursor(210, 82 + 66);
    if(asg[3].ASGoutputOn)
        tft.print("ON");
      else
        {
        tft.setTextColor(ILI9341_PINK);
        tft.print("OFF");
        }
      tft.setTextColor(ILI9341_YELLOW);
      tft.setCursor(255, 82 + 66);
      tft.print("GWN");

  tft.setFont(Arial_9);
  tft.setCursor(10, 170);
  tft.print("Voltages are with a 50-Ohm termination.");

  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);
  }

void tToASGn(void)
  {
  if(currentMenu == 15)       // Coming from SG select
    currentSigGen = menuItem - 1;
  else if(currentMenu==16)    // Coming from SG modify
    {
    switch (menuItem)   // Respond to menu items at the bottom
      {
      case 1: asg[currentSigGen].ASGoutputOn = true;  break;
      case 2: asg[currentSigGen].ASGoutputOn = false;  break;
      case 3:
        if(currentSigGen != 3)  // 3 is a noise gen
          asg[currentSigGen].type = WAVEFORM_SINE;
        break;
      case 4:
        if(currentSigGen != 3)
          asg[currentSigGen].type = WAVEFORM_SQUARE;
        break;
      }
    }

  // Redraw the SG modify screen
  tft.fillRect(0, 0, tft.width(), 200, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(65, 5);
  if(currentSigGen==3)
    tft.print("     NOISE GENERATOR");
  else
    {
    tft.print("SIGNAL GENERATOR");
    tft.setCursor(240, 5);
    tft.print(currentSigGen + 1);  // Internally (0,2), externally (1,3)
    }

  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);

  tft.setCursor(8, 57);
  if(currentSigGen==3)
    ;  //  tft.print("LPF");  TEMP til filter is fixed
  else
    tft.print("Freq");


  tft.setCursor(270, 57);
  tft.print("Hz");

  tft.setCursor(8, 147);
  tft.print("Level");

  // Lower right hand annotation
  tft.fillRect(230, 137, 90, 60, ILI9341_BLACK);

  tft.setCursor(230, 137);
  if(sigGenUnits == VOLTS)
    {

    if(currentSigGen==3)
      {
      tft.print("V into 50R");
      tft.setCursor(240, 159);
      tft.print("1-sigma");
      }
    else
      {
      tft.print("V p-p");
      tft.setCursor(240, 159);
      tft.print("into 50 R");
      }
    }
  // else power units

 /* Touch screen
  * Frequency Up   y = 27 to 52 pixel
  * Frequency Down y = 75 to 100 pixel
  * Level Up       y = 117 to 142 pixel
  * Level Down     y = 165 to 190 pixel
  * All: x=(60, 100)  (100,140)  (140, 180)  (180,220)  (220, 260) pixels
  */
  boxes7(5, 27);  // Draw the up boxes
  //                       num    nDigits xLeft yTop printLeadingZeros color
  printDigits(asg[currentSigGen].freq, 5, 75, 54, false, ILI9341_YELLOW);
  boxes7(5, 75);
  boxes7(4, 117);
  if (asg[currentSigGen].ASGoutputOn)
     printDigits((uint16_t)(1000.0f*asg[currentSigGen].amplitude), 4, 75, 144, true, ILI9341_YELLOW);
  else
     printDigits((uint16_t)(1000.0f*asg[currentSigGen].amplitude), 4, 75, 144, true, ILI9341_PINK);
  tft.setCursor(95, 142);
  tft.setFont(Arial_18);
  tft.print(".");

  boxes7(4, 165);

  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(240, 181);
  if (asg[currentSigGen].type == WAVEFORM_SINE)
    tft.print("Sine");
  else if (asg[currentSigGen].type == WAVEFORM_SQUARE)
    tft.print("Square");
  else if (asg[currentSigGen].type == NOISE)
    tft.print("GWN");

 if(currentSigGen==3) tft.fillRect(0, 25, tft.width(), 75, ILI9341_BLACK); // TEMP for no LPF
  checkSGoverload();
  setSigGens();
  }

void checkSGoverload(void)
  {
  float32_t sumVolts = 0.0f;
  for (int jj=0; jj<4; jj++)
    {
    if(asg[jj].ASGoutputOn)
      sumVolts += asg[jj].amplitude;
    }
  if (instrument==ASA)
   tft.fillRect(15, 0, 60, 15, ILI9341_BLACK);
  if(sumVolts >= 1.2f)
    {
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_12);
    tft.setCursor(230, 183);
    tft.print("OVERLOAD");
    if (instrument==ASA) {
      tft.setCursor(15, 0);
      tft.print("OVERLOAD");
      }
    }


  }

// Set frequency, on/off and amplitude of 3 sig gens plus 1 noise gen
void setSigGens(void)
  {
  // Initialize the three signal generators synth_waveform
  // via "begin(float t_amp, float t_freq, short t_type)"
  mixer2.gain(0, 1.0);   // Turn on signal generatrs
  mixer2.gain(1, 0.0);   // Turn off AVNA source signal
  setRefR(R50);          // 50 Ohm output
  setSwitch(TRANSMISSION_37);        // Connect for T measure
  for (unsigned int ii = 0; ii<3; ii++)
    sgWaveform[ii].begin(uSave.lastState.sgCal*asg[ii].amplitude, factorFreq*asg[ii].freq, asg[ii].type);
  for (unsigned int ii = 0; ii<4; ii++)
    {
    // and the on/off is set by the mixer1 via "void mixer1.gain(unsigned int channel, float gain)"
    if (asg[ii].ASGoutputOn)
      mixer1.gain(ii, 1.0f);
    else
      mixer1.gain(ii, 0.0f);
    }
  noise1.amplitude(asg[3].amplitude);
  noise1.setLowPass(factorFreq*asg[3].freq);
  }

void tToAVNA(void)
  {
  // If VVM was used last, the VNA oscillators need to be reset
  if (instrument == VVM)
    {
    FreqData[0].freqHz = saveFreq0;
    setUpNewFreq(0);
    }
  instrument = AVNA;
  mixer2.gain(0, 0.0f);   // Turn off signal generatrs
  mixer2.gain(1, 1.0f);   // Turn on AVNA source signal
  }

// Vector Voltmeter
void tToVVM(void)
  {

  if(currentMenu==17)    // Coming from VVM modify
    {
    switch (menuItem)   // Respond to menu items at the bottom
      {
      case 1: phaseOffset = 0.0f;  break;
      case 2: VVMFormat = VVM_VOLTS;  break;
      case 3: VVMFormat = VVM_DBM;  break;
      }
    }

  // For VVM the AVNA freq source is set to same freq as Sig Gen1
  FreqData[0].freqHz = asg[0].freq;
  // VVM uses the VNA routines and oscillator
  setUpNewFreq(0);

  // Resync the phase of the three sources
  AudioNoInterrupts();
  //waveform1.begin(WAVEFORM_SINE);
  waveform1.frequency(factorFreq * asg[0].freq);
  waveform1.amplitude(1.0);
  waveform1.phase(0.0f);
  //waveform2.begin(WAVEFORM_SINE);
  waveform2.frequency(factorFreq * asg[0].freq);
  waveform2.amplitude(1.0);
  waveform2.phase(270.0f);
  //sgWaveform[0].begin(asg[0].type);
  sgWaveform[0].frequency(factorFreq * asg[0].freq);
  sgWaveform[0].amplitude(asg[0].amplitude);
  sgWaveform[0].phase(0.0f);
  AudioInterrupts();

  // The left ('Z') port comes from the four sig gens.  They can be turned off
  // in the SG menu if not wanted.  Sig Gen 1 controls the frequency of the
  // VVM, even if it is turned off.  For steady phase, Sig Gen 1 should be
  // used.  To measure voltage, or dBm, the frequency needs to be "close".
  setSigGens();
  //mixer2.gain(0, 1.0f);   // Turn on signal generators
  //mixer2.gain(1, 0.0f);   // Turn off AVNA source signal
  instrument = VVM;
  mixer2.gain(0, 1.0f);   // Turn on 4x signal generatrs
  mixer2.gain(1, 0.0f);   // Turn off AVNA source signal
  tft.fillRect(0, 0, tft.width(), 200, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(65, 3);
  tft.print("VECTOR VOLTMETER");

  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);

  if (asg[0].ASGoutputOn)
    tft.setTextColor(ILI9341_WHITE);
  else
    tft.setTextColor(ILI9341_PINK);
  tft.setFont(Arial_9);
  tft.setCursor(5, 28);
  tft.print("Sig Gen #1:");
  tft.setCursor(75, 28);
  tft.print(asg[0].freq, 0);
  tft.setCursor(110, 28);
  tft.print("Hz   Level V p-p =");
  tft.setCursor(220, 28);
  tft.print(asg[0].amplitude, 3);

  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);

  tft.setCursor(36, 50);
  if (VVMFormat == VVM_VOLTS)
    tft.print("Volts RMS");
  else if(VVMFormat == VVM_DBM)
    tft.print("Power dBm");
  tft.setCursor(180, 50);
  tft.print("Phase, Deg");

  tft.setCursor(230, 120);
  tft.print("Phase");
  tft.setCursor(230, 142);
  tft.print("Offset");
  tft.setCursor(230, 164);
  tft.print("Degrees");

 /* Touch screen
  * Phase Up       y = 117 to 142 pixel
  * Phase Down     y = 165 to 190 pixel
  * All: x=(60, 100)  (100,140)  (140, 180)  (180,220)  (220, 260) pixels
  */
  boxes7(4, 117);
  printDigits((uint16_t)(10.0f*fabsf(phaseOffset)), 4, 75, 144, true, ILI9341_YELLOW);
  tft.setCursor(175, 142);
  tft.setFont(Arial_18);
  tft.print(".");
  if (phaseOffset < 0.0f)
    {
    tft.setCursor(60, 142);
    tft.print("-");
    }
  boxes7(4, 165);

  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(260, 168);
  }

void VVMMeasure(void)
  {
  float32_t MT, PT, MTdB;

  getFullDataPt();
  checkOverload();
  MT = uSave.lastState.VVMCalConstant*(float32_t)amplitudeV; // V rms at terminals
  MTdB = 13.01056f + 20.0f * log10f(MT);  // 50 Ohm, Vrms to dBm
  PT = phaseOffset + (float32_t)phaseV;
  if (PT>180.0f)
    PT -= 360.0f;
  else if (PT<-180.0f)
    PT += 360.0f;

  tft.fillRect(0, 70, tft.width(), 45, ILI9341_BLACK);
  if (pkDet.available())
    {
    float32_t pk = pkDet.readPeakToPeak();
    if (pk > 1.95)
      tft.setTextColor(ILI9341_RED);
    else
      tft.setTextColor(ILI9341_WHITE);
    //tft.fillRect(270, 28, 50, 15, ILI9341_BLACK);
    tft.setFont(Arial_8);
    tft.setCursor(5, 100);
    if (VVMFormat == VVM_VOLTS)
      tft.print("   V RMS at fundamental freq");
    else
      tft.print("   Power at fundamental freq");
    tft.setCursor(200,100);
    tft.print("ADC %p-p=");
    tft.setCursor(265, 100);
    tft.print(50.0f*pk, 1);
    }

  // TFT Print volts and phase
  tft.setFont(Arial_24);
  tft.setCursor(25, 70);
  if (VVMFormat == VVM_VOLTS)
    tft.print(MT, 6);
  else if (VVMFormat == VVM_DBM)
    tft.print(MTdB, 2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(190, 70);
  tft.print(PT, 2);

  tft.fillRect(270, 28, 50, 15, ILI9341_BLACK);
  tft.setFont(Arial_9);
  tft.setCursor(270, 28);
  tft.print(countMeasurements);
  }

void tToASA(void)
  {
  static bool beenHere = false;
/*  6000.0f, S6K, 2.5f, "6 kHz", 16,
    12000.0f, S12K, 5.0f, "12 kHz", 16,
    24000.0f, S24K, 10.0f, "24 kHz", 32,
    48000.0f, S48K, 20.0f, "48 kHz", 64,
    96000.0f, S96K, 40.0f, "96 kHz", 128,
    192000.0f, S192K, 80.0f, "192 kHz", 200
*/
  if(!beenHere)
    {
    beenHere = true;
    ASAI2SFreqIndex = 4;  // 96 kHz
    }
  instrument = ASA;
  mixer2.gain(0, 1.0f);   // Turn on 4x signal generatrs
  mixer2.gain(1, 0.0f);   // Turn off AVNA source signal

  setSwitch(TRANSMISSION_37);
  DC1.amplitude(0.0);   // (dacLevel);
  tft.fillScreen(ILI9341_BLACK);
  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);
  setSample(freqASA[ASAI2SFreqIndex].rateIndex);
  countMax = freqASA[ASAI2SFreqIndex].SAnAve;
  prepSpectralDisplay();
  writeMenus(3);
  }

void tToASAFreq(void)
  {
  instrument = ASA_IDLE;
  // 192 kHz sample rate runs out of power and has spectral errors.
  // Allow up to 96 kHz
  if((currentMenu == 13 ) && (menuItem == 1) && (ASAI2SFreqIndex<4))
      ASAI2SFreqIndex++;
  if((currentMenu == 13 ) && (menuItem == 2) && (ASAI2SFreqIndex>0))
      ASAI2SFreqIndex--;
  if((currentMenu == 13 ) && (menuItem == 3))
        freqASA[ASAI2SFreqIndex].SAnAve *= 2;
  if( (currentMenu == 13 ) && (menuItem == 4) && (freqASA[ASAI2SFreqIndex].SAnAve >= 4) )
        freqASA[ASAI2SFreqIndex].SAnAve /= 2;
  
  setSample(freqASA[ASAI2SFreqIndex].rateIndex);
  countMax = freqASA[ASAI2SFreqIndex].SAnAve;
  tft.fillRect(0, 38, tft.width(), 200, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print(" ");
  tft.setCursor(20, 60);
  tft.print("Sample Rate for Spectrum Analyzer");
  tft.setCursor(120, 82);
  tft.setFont(Arial_16);
  tft.print(freqASA[ASAI2SFreqIndex].name);
  tft.setFont(Arial_12);
  tft.setCursor(20, 120);
  tft.print("Maximum Freq, kHz");
  tft.setCursor(220, 120);
  tft.print(freqASA[ASAI2SFreqIndex].maxFreq, 1);
  tft.setCursor(20, 142);
  tft.print("Resolution Bandwidth, Hz ");
  tft.setCursor(220, 142);
  tft.print(0.0015*freqASA[ASAI2SFreqIndex].sampleRate, 1);   // Incl Hann window effects
  tft.setCursor(20, 164);
    tft.print("Samples per Update");
  tft.setCursor(220, 164);
  tft.print(freqASA[ASAI2SFreqIndex].SAnAve); 
  }

void tToASAAmplitude(void)
  {
  instrument = ASA_IDLE;
  // Allow dB/div as 5, 10, 20.
  if((currentMenu == 14 ) && (menuItem == 1) && (dbPerDiv<20.0f))  // dB/div up
    {
    if(dbPerDiv < 9.0f)  dbPerDiv = 10.0f;
    else  dbPerDiv = 20.0f;
    }
  if((currentMenu == 14 ) && (menuItem == 2) && (dbPerDiv > 5.0f))
    {
    if(dbPerDiv > 11.0f) dbPerDiv = 10.0f;
    else dbPerDiv = 5.0f;
    }
  // Allow dBOffset in 5 dB steps
  if((currentMenu == 14 ) && (menuItem == 3))  // dBOffset up
    ASAdbOffset += 5.0f;
  if((currentMenu == 14 ) && (menuItem == 4))
    ASAdbOffset -= 5.0f;

  tft.fillRect(0, 38, tft.width(), 200, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print(" ");
  tft.setCursor(20, 60);
  tft.print("Scale, dB per Division =");
  tft.setCursor(210, 58);
  tft.setFont(Arial_16);
  tft.print(dbPerDiv, 2);

  tft.setFont(Arial_12);
  tft.setCursor(20, 90);
  tft.print("Y-axis Offset in dB =");
  tft.setCursor(210, 88);
  tft.setFont(Arial_16);
  tft.print(ASAdbOffset,2  );
  }

void tToASASinad(void) {
  if(sinadOn) {     // So, we turning Sinad off
    sinadOn = false;
    ASAI2SFreqIndex = sinadLastRate; // restore
    setSample(ASAI2SFreqIndex);
    prepSpectralDisplay();
    show_spectrum();
    }
  else {
    sinadOn = true;
    sinadLastRate = ASAI2SFreqIndex;  // Save for restore
    ASAI2SFreqIndex = S12K;
    setSample(ASAI2SFreqIndex);
    prepSpectralDisplay();
    show_spectrum();
    }

uint16_t sinadLastRate = S96K;
}

// This may be extreme, but does the job.  Clears reference to data
// that may be out of date.
void clearStatus(void)
  {
  dataValidTSweep = false;
  dataValidZSweep = false;
  calZSingle = false;
  calTSingle = false;
  calZSweep = false;
  calTSweep = false;
  }

void tDoSingleFreq(void)
  {
  avnaState = 0;
  topLines();
  tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(20, 62);
  tft.print("Single Frequency Measurements");
  clearStatus();
  }

void tCalCommand(void)
  {
  topLines();
  tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(20, 60);
  tft.print("Wait for Cal to complete...");
  // The next 3 are redundant for sweep, but needed for single freq
  setUpNewFreq(nFreq);
  DC1.amplitude(dacLevel);     // Turn on sine wave
  delay(50);
  CalCommand();
  tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
  // After cal, but before measurement, data is no longer valid for display
  dataValidTSweep = false;
  dataValidZSweep = false;
  }

void tDoSweep(void)
  {
  avnaState = 0;
  topLines();
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_14);
  tft.setCursor(20, 62);
  tft.print("Swept 13-Freq Measurements");
  displaySweepRange();
  }


// This is for touch-LCD controlled sweep. It is a single sweep, commanded
// by "Single Sweep" on touch LCD. The display is separate, as it needs to change
/// withthe displayed frequencies.
void tDoSweepZ(void)                        // Impedance Sweep
  {
  setRefR(uSave.lastState.iRefR);
  lcdTitle("Sweep Z");
  tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
  uSave.lastState.SingleorSweep = SWEEP;
  uSave.lastState.ZorT = IMPEDANCE;
  if(calZSweep == false)
     {
     tft.setTextColor(ILI9341_YELLOW);
     tft.setFont(Arial_12);
     tft.setCursor(20, 60);
     tft.print("Wait - Doing Z-Sweep Cal");
     CalCommand();
     }
  avnaState = ZSWEEP;
  if(calZSweep)
     {
     DC1.amplitude(dacLevel);     // Turn on sine wave   <<<<<<<<<MAKE PROGRAMMABLE
     setSwitch(IMPEDANCE_38);
     tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
     tft.setTextColor(ILI9341_YELLOW);
     tft.setFont(Arial_12);
     tft.setCursor(20, 60);
     tft.print("Wait - Doing Z-Sweep Measure");
     for (nFreq = 1; nFreq <= 13;  nFreq++) // Over all frequencies; nFreq is global freq index
        {
        setUpNewFreq(nFreq);
        delay(ZDELAY + (unsigned long)(1000.0 / FreqData[nFreq].freqHz));
        measureZ(nFreq);   // result is Z[nFreq], Y[nFreq], sLC[nFreq], pLC[nFreq] and Q[nFreq]
        serialPrintZ(nFreq);    // Rev 0.6.0
        }
     nFreq = 0;
     setUpNewFreq(nFreq);
     tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
     dataValidZSweep = true;
     display7Z();
     }
  }

// This is for touch-LCD controlled sweep. It is a single sweep, commanded
// by "Single Sweep" on touch LCD. The display is separate, as it needs to change
/// withthe displayed frequencies.
void tDoSweepT(void)
  {
  topLines();
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  uSave.lastState.SingleorSweep = SWEEP;
  uSave.lastState.ZorT = TRANSMISSION;
  if(calTSweep == false)
    {
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_14);
    tft.setCursor(20, 100);
    tft.print("T - Sweep 13-Freq  NEEDS CAL");
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_12);
    tft.setCursor(20, 130);
    tft.print("Connect reference through path");
    tft.setCursor(20, 152);
    tft.print("and hit Cal below.");
    }
  avnaState = TSWEEP;
  if(calTSweep)
     {
     DC1.amplitude(dacLevel);     // Turn on sine wave  <<<MAKE PROGRAMMABLE
     tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
     tft.setTextColor(ILI9341_YELLOW);
     tft.setFont(Arial_12);
     tft.setCursor(20, 60);
     tft.print("Wait for Sweep to complete...");
     for (nFreq = 1; nFreq <= 13;  nFreq++) // Over all frequencies; nFreq is global freq index
        {
        setUpNewFreq(nFreq);
        delay(ZDELAY + (unsigned long)(1000.0 / FreqData[nFreq].freqHz));
        measureT();      // result is Tmeas
        T[nFreq] = Tmeas;
        }
     tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
     dataValidTSweep = true;
     display7T();
     nFreq = 0;  // Back in range
     setUpNewFreq(nFreq);
     }
  }

// Intro screen for What? function
void tWhatIntro(void)
  {
  lcdTitle("What?");
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_11);
  tft.setCursor(10, 50);
  tft.print("         WHAT?  Component Identification");
  tft.setCursor(10, 80);
  tft.print("Component impedance will be measured at");
  tft.setCursor(10, 102);
  tft.print("13 frequencies and 2 impedance levels.");
  tft.setCursor(10, 124);
  tft.print("Based on these, values are estimated.");
  tft.setCursor(10, 146);
  tft.print("Omit 3 lowest frequencies for faster estimates.");
  tft.setCursor(10, 168);
  tft.print("Connect component and touch \"Search Value\"");
  }

// tDoWhatPart() is response to touching LCD.  The next function doWhatPart()
// is called from loop() to keep menus correct.
void tDoWhatPart(void)
  {
  avnaState=WHATSIT;
  }

// Does two Z sweeps, one at 50 Ohms and one at 5KOhm.
// Chooses frequencies with Z magnitude closest to 50 or 5K
// and prints part values.  Q>1.0 are L & C, otherwise R's.
void doWhatPart(void)
  {
  uint16_t i, nBest, baseIndex, iRefRSave, saveZT, saveSS, savenFreq;
  double ZM, ZMBest, zDiff;
  double XX, RR;

  // Leave this function with the sate restored
  iRefRSave = uSave.lastState.iRefR;
  saveSS = uSave.lastState.SingleorSweep;
  saveZT = uSave.lastState.ZorT;
  savenFreq = nFreq;

  if(whatUseLF)  baseIndex = 1;
  else           baseIndex = 4;

  // avnaState = 5;
  tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 60);
  tft.print("     WAIT  -  Searching out component Z");
  uSave.lastState.iRefR = R50;
  setRefR(R50);            // Search out 50 Ohms first
  uSave.lastState.SingleorSweep = SWEEP;
  uSave.lastState.ZorT = IMPEDANCE;

  CalCommand0(baseIndex);
  setSwitch(IMPEDANCE_38);
  doZSweep(false, baseIndex);              // Result in Z[nf]
  nBest = 1;
  zDiff = 1E9;
  ZMBest = 0.0;
  for(i=baseIndex; i<=13; i++)
     {
     RR = Z[i].real();
     XX = Z[i].imag();
     ZM = sqrt(RR*RR + XX*XX);
     if(fabs(50.0 - ZM) < zDiff)
        {
        zDiff = fabs(50.0 - ZM);
        nBest = i;
        ZMBest = ZM;
        }
     }

    tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
    tft.setCursor(10, 40);
    tft.setFont(Arial_14);
    tft.print("Ref=50 Ohm, At Freq = ");
    if     (FreqData[nBest].freqHz < 100.0)
       tft.print(FreqData[nBest].freqHz, 2);
    else if(FreqData[nBest].freqHz < 1000.0)
       tft.print(FreqData[nBest].freqHz, 1);
    else    //  Must be 1000 or more
       tft.print(floor(0.5+FreqData[nBest].freqHz), 0);
    tft.print(" Hz:");

    RR = Z[nBest].real();
    XX = Z[nBest].imag();
    tft.setFont(Arial_18);
    tft.setCursor(20, 67);
    if(XX<=0.0 && (Q[nBest]>1.0 || Q[nBest]<0.0))          // Call it a capacitor
       {
       tft.print("C=");
       tft.print(valueString(sLC[nBest], cUnits));
       tft.print("  Q=");
       // Serial.print shows negative Q's as a number, +9999.9.  Here we can be less terse:
       if(Q[nBest]<9999.8 && Q[nBest]>1.0)
          tft.print(Q[nBest], 1);
       else
          tft.print(" High");
       }
    else if(XX>0.0 && (Q[nBest]>1.0 || Q[nBest]<0.0))      // Call it an inductor
       {
       tft.print("L=");
       tft.print(valueString(sLC[nBest], lUnits));
       tft.print("  Q=");
       if(Q[nBest] < 9999.8 && Q[nBest]>1.0)
          tft.print(Q[nBest], 1);
       else
          tft.print(" High");
       }
    else     // Low Q, show as a resistor with series L or C
       {
       tft.print("R=");
       tft.print(valueStringSign(RR, rUnits));
       if(XX > 0.0)
          {
          tft.print(" L=");
          tft.print(valueString(sLC[nBest], lUnits));
          }
       else
          {
          tft.print(" C=");
          tft.print(valueString(sLC[nBest], cUnits));
          }
        }
     printQuality(ZMBest / 50.0);

  // This is mostly duplicate stuff of 50 Ohm---make separate function <<<<<<<<<<<
  tft.setFont(Arial_12);
  tft.setCursor(10, 140);
  tft.print("  WAIT - Searching out Z for 5K Ref");
  uSave.lastState.iRefR = R5K;
  setRefR(R5K);            // Search out 5K Ohms
  CalCommand0(baseIndex);
  setSwitch(IMPEDANCE_38);
  doZSweep(false, baseIndex);              // Result in Z[nf]
  nBest = 1;
  zDiff = 1E9;
  for(i=baseIndex; i<=13; i++)
     {
     RR = Z[i].real();
     XX = Z[i].imag();
     ZM = sqrt(RR*RR + XX*XX);
     if(fabs(5000.0 - ZM) < zDiff)
        {
        zDiff = fabs(5000.0 - ZM);
        nBest = i;
        ZMBest = ZM;
        }
     }
    tft.fillRect(0, 100, tft.width(), 84, ILI9341_BLACK);
    tft.setCursor(10, 120);
    tft.setFont(Arial_14);
    tft.print("Ref=5K Ohm, At Freq = ");
    if     (FreqData[nBest].freqHz < 100.0)
       tft.print(FreqData[nBest].freqHz, 2);
    else if(FreqData[nBest].freqHz < 1000.0)
       tft.print(FreqData[nBest].freqHz, 1);
    else    //  Must be 1000 or more
       tft.print(floor(0.5+FreqData[nBest].freqHz), 0);
    tft.print(" Hz:");

    RR = Z[nBest].real();
    XX = Z[nBest].imag();

    tft.setFont(Arial_18);
    tft.setCursor(20, 147);

    if(XX<=0.0 && (Q[nBest]>1.0 || Q[nBest]<0.0))          // Call it a capacitor
       {
       tft.print("C=");
       tft.print(valueString(sLC[nBest], cUnits));
       tft.print("  Q=");
       // Serial.print shows negative Q's as a number, +9999.9.  Here we can be less terse:
       if(Q[nBest] < 9999.8 && Q[nBest]>1.0)
          tft.print(Q[nBest], 1);
       else
          tft.print(" High");
       }
    else if(XX>0.0 && (Q[nBest]>1.0 || Q[nBest]<0.0))      // Call it an inductor
       {
       tft.print("L=");
       tft.print(valueString(sLC[nBest], lUnits));
       tft.print("  Q=");
       if(Q[nBest] < 9999.8 && Q[nBest]>1.0)
          tft.print(Q[nBest], 1);
       else
          tft.print(" High");
       }
    else     // Low Q, show as a resistor with series L or C
       {
       tft.print("R=");
       tft.print(valueStringSign(RR, rUnits));
       if(XX > 0.0)
          {
          tft.print(" L=");
          tft.print(valueString(sLC[nBest], lUnits));
          }
       else
          {
          tft.print(" C=");
          tft.print(valueString(sLC[nBest], cUnits));
          }
        }
  printQuality(ZMBest / 5000.0);

  // Leave this function with the sate restored
  uSave.lastState.SingleorSweep = saveSS;
  uSave.lastState.ZorT = saveZT;
  nFreq = savenFreq;
  uSave.lastState.iRefR = iRefRSave;
  setRefR(uSave.lastState.iRefR);
  }

// Simple measurement quality estimate. Ratio is impedance/refR.
void printQuality(float32_t ratio)
  {
  if(ratio < 1.0f)   ratio = 1.0 / ratio;
  if(ratio < 10.0f)        tft.print("  E");
  else if(ratio < 100.0f)  tft.print("  G");
  else if(ratio < 1000.0) tft.print("  P");
  else                    tft.print("  0");
  }

// Low frequencies of 10, 20 and 50 Hz are especially slow to execute
// Z measurements.  For What? function, their use is optional.
void tUseLF(void)
  {
  whatUseLF = true;
  tft.fillRect(80, 185, tft.width(), 14, ILI9341_BLACK);
  }
void tNoLF(void)
  {
  whatUseLF = false;
  tft.fillRect(80, 185, tft.width()-80, 14, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_8);
    tft.setCursor(145, 185);
    tft.print("Not Using 10, 20 or 50 Hz");
  }

void tDoSettings(void)
  {
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);

  tft.setFont(Arial_12);
  tft.setCursor(0, 60);
  tft.print("   Currently, Ref R = "); tft.print(uSave.lastState.valueRRef[uSave.lastState.iRefR]);
          tft.print(" Ohms");
  tft.setCursor(0, 90);
  tft.print("  Select \"50\" or \"5K\" Ref R on menu below");
  }

void tSet50(void)
  {
  sRefR(R50);
  tft.print("  Ref R = 50 Ohms");
  }

void tSet5K(void)
  {
  sRefR(R5K);
  tft.print("  Ref R = 5000 Ohms");
  }

// All the common stuff for setting the reference resistance
void sRefR(int16_t indexRef)
  {
  uSave.lastState.iRefR = indexRef;   // Where we keep track of RefR
  setRefR(indexRef);           // Set relay
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 100);
  tft.print(" Select \"Back\" for Main Menu");
  topLines();
  saveStateEEPROM();
  clearStatus();
  // Get ready for printing "Ref R = 50 Ohms" etc
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_24);
  tft.setCursor(0, 60);
  }

void tDoHelp(void)
  {
  avnaState = 6;
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print("HELP-Fixed Freq Measurements, Z or T");
  tft.setCursor(0, 62);
  tft.setFont(Arial_11);
  tft.print("Touch \"Single Freq\", select freq with 3 buttons");
  tft.setCursor(0, 86);
  tft.print("Touch \"Set Ref R\" and choose 50 or 5K Ohm");
  tft.setCursor(0,110);
  tft.print("Select \"Meas Z\" or T");
  tft.setCursor(0, 134);
  tft.print("Cal for Z is automatic, T is manual.");
  tft.setCursor(0, 158);
  tft.print("Measurements will repeat. Return with \"Back\"");
  tft.setCursor(0, 182);
  tft.print("HELP-What? Touch \"What?\"; follow instructions.");
  tft.setFont(Arial_12);
  }
void tDoHelp2(void)
  {
  avnaState = 6;
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print("HELP-Swept Freq Measurements, Z & T");
  tft.setFont(Arial_11);
  tft.setCursor(0, 62);
  tft.print("Touch \"Set Ref R\" and choose 50 or 5K Ohm.");
  tft.setCursor(0, 86);
  tft.print("Touch \"Back\" and then \"Sweep\".");
  tft.setCursor(0, 110);
  tft.print("Touch \"Meas Z\" or \"Meas T\".");
  tft.setCursor(0, 134);
  tft.print("Z measurements proceed but T requires");
  tft.setCursor(0, 158);
  tft.print("  Cal. Follow prompts.");
  tft.setCursor(0, 182);
  tft.print("Use \"Disp Freq\" to scroll freq range up/down.");
  tft.setFont(Arial_12);
  }

void tFreqUp(void)
  {
  //Up if not at top, print top line
  if(nFreq==0)
     nFreq = 8;
  else if(nFreq<13)
     nFreq++;
  finish_tFreq();
  }

void tFreqDown(void)
  {
  if(nFreq==0)
     nFreq = 6;
  else if(nFreq>1)
     nFreq--;
  finish_tFreq();
  }

void tFreq0(void)
  {
  nFreq = 0;
  setUpNewFreq(nFreq);
  finish_tFreq();
  clearStatus();
  }

void finish_tFreq(void)
  {
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_24);
  tft.setCursor(0, 60);
  tft.print("Freq = ");  tft.print(FreqData[nFreq].freqHz, 0); tft.print(" Hz");
  saveStateEEPROM();
  topLines();
  calZSingle = false;
  calTSingle = false;
  }

void tDoSingleZ(void)
  {
  setRefR(uSave.lastState.iRefR);
  lcdTitle("Single Freq Z");
  uSave.lastState.SingleorSweep = SINGLE;
  uSave.lastState.ZorT = IMPEDANCE;
  if(calZSingle == false)
    {
    setUpNewFreq(nFreq);
    DC1.amplitude(dacLevel);     // Turn on sine wave
    delay(50);
    tCalCommand();
    calZSingle = true;
    }
  // Actual measurement will occur in loop().  Here we enable continuous measurement
  avnaState = ZSINGLE;
  }

void lcdTitle(char const *title)
  {
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_11);
  tft.setCursor(10, 185);
  tft.print(title);
  }

void tDoSingleT(void)
  {
  lcdTitle("Single Freq T");
  uSave.lastState.SingleorSweep = SINGLE;
  uSave.lastState.ZorT = TRANSMISSION;
  if(calTSingle == false)
    {
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_14);
    tft.setCursor(20, 100);
    tft.print("T - Single Freq  NEEDS CAL");
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_12);
    tft.setCursor(20, 130);
    tft.print("Connect reference through path");
    tft.setCursor(20, 152);
    tft.print("and hit Cal below.");
    }
  // Actual measurement will occur in loop().  Here we enable continuous measurement
  avnaState = TSINGLE;
  }

void tSetParams(void)
  {
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
 // tft.setTextColor(ILI9341_YELLOW);
  //tft.setFont(Arial_24);
  //tft.setCursor(0, 60);
 // tft.print(" 1 Z ");
  }

void tSweepFreqDown(void)
  {
  if((nFreqDisplay--) <= 1)
      nFreqDisplay++;
  if     (dataValidTSweep)
     display7T();
  else if(dataValidZSweep)
     display7Z();
  else
     displaySweepRange();
  }

void tSweepFreqUp(void)
  {
  if((nFreqDisplay++) >= 7)
      nFreqDisplay--;
  if     (dataValidTSweep)
     display7T();
  else if(dataValidZSweep)
     display7Z();
  else
     displaySweepRange();
  }

void tNothing(void)
  {
  // Typically here from a blank menu item.
  }

/* boxes7(nBox, yTop)
 * Up and down boxes to change values of a number, e.g.,
 * the frequency and amplitude of the signal generators.  There is
 * room for 7 boxes, 25 pixels high and 38 pixels wide.  Before the boxes
 * are drawn, the entire araeis cleared to allow the fill1o7() to add a fill
 * to one of the boxes (such as feedback for touching a box).
 */
void boxes7(uint16_t nBox, int16_t yTop)
  {
  switch (nBox)
    {
    case 4:
      tft.fillRect(60, yTop+1, 160, 25, ILI9341_BLACK);
      for (int ii=1; ii<5; ii++)
        tft.drawRect(21+40*ii, yTop, 38, 25, ILI9341_WHITE);
      break;

    case 5:
      tft.fillRect(60, yTop+1, 200, 25, ILI9341_BLACK);
      for (int ii=1; ii<6; ii++)
        tft.drawRect(21+40*ii, yTop, 38, 25, ILI9341_WHITE);
      break;

    case 7:
      tft.fillRect(20, yTop+1, 280, 25, ILI9341_BLACK);
      for (int ii=0; ii<8; ii++)
        tft.drawRect(21+40*ii, yTop, 38, 25, ILI9341_WHITE);
      break;

    default:
      break;
    }
  }

 void fill1of7(uint16_t nBox, int16_t yTop, uint16_t fillColor)
   {
   tft.fillRect(22+40*nBox, yTop+1, 36, 23, fillColor);
   }

 void printDigits(uint16_t num, uint16_t nDigits, int16_t xLeft, int16_t yTop, bool printLeadingZeros, uint16_t color)
   {
   uint16_t nm, digitPosition;

   tft.fillRect(xLeft-20, yTop, 40*nDigits, 20, ILI9341_BLACK);
   tft.setTextColor(color);
   tft.setFont(Arial_18);
   digitPosition = nDigits;  // Work right to left
   if(num==0 && !printLeadingZeros)
     {
     tft.setCursor(xLeft + (nDigits - 1)*40, yTop);
     tft.print("0");
     }
   else
     {
     for(int ii=0; ii<nDigits; ii++)
       {
       nm = num - 10*(num/10);
       num = num/10;
       //print nm
       tft.setCursor(xLeft + (nDigits - ii - 1)*40, yTop);
       if(nm>0 || num>0 || printLeadingZeros)
         tft.print(nm, 1);
       else
         tft.print(" ");

       if(--digitPosition < 0)
         break;
       }
     }
   }

void writeMenus(uint16_t nMenu)
  {
  uint16_t i;
  // Room for 1 menu sets of 6 menus each, and each menu has up to 8 chars + 0.
  static char line1[22][6][9]={
    " ", "  AVNA", " Vector", "Spectrum", " Signal", " Service", // Set 0
    "Instrmnt", " Single", " Sweep", "  What", "  Set", " Help", // Set 1
    " Back", "  Freq", "  Freq", "  Freq", " Meas", " Meas ",    // Set 2
    " Back", "Disp Frq", "Disp Frq", " ", " Meas", " Meas",      // Set 3
    " Back", "    50", "    5K", " ", " ", " ",                  // Set 4
    " Back", " 50 Ohm", " 5K Ohm", " Power", " Power", " ",      // Set 5
    " Back", " Screen", " Screen", " ", " ", " ",                // Set 6
    " Back", " Screen", " Screen", " ", " ", " ",                // Set 7 Help
    " Back", " ", " ", "  No 10,", " Use 10,", " Search",        // Set 8 What?
    " Back", " ", " ", " ", "  Cal", " Meas",                    // Set 9
    " Back", "Disp Frq", "Disp Frq", " ", "  Cal", " Single",    // Set 10
    " Back", "Disp Frq", "Disp Frq", " ", "  Cal", " Single",    // Set 11
    "Instrmnt", " Freq", "Amplitde", " SINAD ", " ", " ",        // Set 12 ASA Home
    " Back", "Max Freq", "Max Freq", "Samples", "Samples ", " ", // Set 13 ASA Freq
    " Back", " dB/div", " dB/div", " Offset ", " Offset ", " ",  // Set 14 ASA Amplitude
    "Instrmnt", " SigGen", " SigGen", " SigGen", "NoiseGen", " ",// Set 15 ASG Home
    " Back", "Sig Gen", "Sig Gen", "  Sine ", " Square ", " ",   // Set 16 ASG 1 to 4
    " Back", " Reset", " Volts", "  dBm   ", "   ", " ",         // Set 17 VVM
    "Instrmnt", " Touch", "V Input", "V Output", " ", " ",       // Set 18 Service
    "   Cal", " ", " ", " ", "  ", " ",                          // Set 19 Touch Cal
    "   Cal", " Measure", " ", " ", "  ", " Cancel",             // Set 20 Input V Cal
    "   Cal", " Measure", " ", " ", "  ", " Cancel"              // Set 21 Output V Cal
    };
  static char line2[22][6][9]={
    " ", " ", " VMeter", "Analyzer", "  Gens", " & Cal ",
    " Home", "  Freq", " ", "   ?", " Ref R", " ",
    " ", "  Down", "   Up", "    0", "    Z", "    T",
    " ", "  Down", "   Up", " ", "    Z", "    T",
    " ", "  Ohms", "   Ohms ", " ", " ", " ",
    " ", " ", " ", "    Up", " Down", " ",
    " ", "    1", "    2", " ", " ", " ",
    " ", "    1", "    2", " ", " ", " ",
    " ", " ", " ", "20 or 50", "20 & 50", " Value",
    " ", " ", " ", " ", " ", "    T",
    " ", "  Down", "   Up", " ", " ", "T Sweep",
    " ", "  Down", "   Up", " ", " ", "Z Sweep",
    " Home", " ", " ", " Toggle ", " ", " ",
    " ", "   Up ", "  Down ", "   Up", "  Down ", " ",
    " ", "   Up ", "  Down ", "    Up ", "  Down ", " ",
    " Home", "    1", "    2", "    3", "    4", " ",
    " ", "  ON", "  OFF", " Wave ", " Wave ", " ",
    " ", " Phase ", " High-Z", " 50-Ohm ", " ", " ",
    " Home", "  Cal ", "  Cal ", "  Cal ", " ", " ",
    " Done", " ", " ", " ", " ", " ",
    " Done", " ", " ", " ", " ", " ",
    " Done", " ", " ", " ", " ", " "
    };

    tft.fillRect(0, 200, tft.width(), 39, ILI9341_BLACK);
  for(i=0; i<6; i++)
    {
    tft.drawRect(53*i, 200, 51, 39, ILI9341_GREEN);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_8);
    tft.setCursor(53*i+5, 209);
    tft.print(&line1[nMenu][i][0]);
    tft.setCursor(53*i+5, 222);
    tft.print(&line2[nMenu][i][0]);
    }
  }
