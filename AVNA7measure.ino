// VNA measurements for the AVNA  - made mdule 24 Jan 2020  RSL

// doZSingle() measures impedance at a single frequency. The calling
// parameter, out sets Serial.print.  nFreq sets the frequency.
void doZSingle(bool out)
  {
  // Check that cal done, settings OK

  // As of ver0.70 there is always cal available from EEPROM using CALSAVE.  So:
#if 0
  if (!calZSingle && !(doRun==SINGLE_NC))
     {
     Serial.println("Single frequency measurements without Cal---use CAL or C");
     doRun = RUNNOT;
     return;
     }
#endif

  DC1.amplitude(dacLevel);     // Turn on sine wave
  setUpNewFreq(nFreq);
  delay(ZDELAY);   // Delay until level is constant
  measureZ(nFreq);
  // if (useUSB)
  serialPrintZ(nFreq);
  // if(useLCD)
  LCDPrintSingleZ(nFreq);
  }

// doZSweep() measures impedance at all sweep frequencies. The calling
// parameter, out, sets Serial.print.
void doZSweep(bool out, uint16_t iStart)
  {
#if 0
  if (!calZSweep)
    {
    if(out)
       Serial.println("Sweep cannot run without Cal---use CAL or C");
    doRun = RUNNOT;
    return;
    }
#endif
  DC1.amplitude(dacLevel);     // Turn on sine wave
  // Add column headings if not annotated
  if(!annotate && seriesRX && !parallelRX)
    Serial.println(" Freq,  R,  X,  L or C,  Q");
  if(!annotate && parallelRX && !seriesRX)
    Serial.println(" Freq,  G,  B,  Res,  L or C,  Q");
  for (nFreq = iStart; nFreq <= 13;  nFreq++) // Over all frequencies; nFreq is global freq index
    {
    setUpNewFreq(nFreq);
    delay(ZDELAY);        // Delay until level is constant
    measureZ(nFreq);      // result is Z[nFreq]
    serialPrintZ(nFreq);
    }
  nFreq = 0;    // Don't leave at 14!
  setUpNewFreq(nFreq);
  }

// =========================  TRANSMISSION  =========================

// doTSingle() measures transmission at single frequency, set by global nFreq.
void doTSingle(bool out)
  {

  // ver0.70 Cal is always available from EEPROM, so:
#if 0
  if (!calTSingle && out)
    {
    Serial.println("Single frequency measurements without Cal---use CAL or C");
    doRun = RUNNOT;
    return;
    }
#endif

  DC1.amplitude(dacLevel);     // Turn on sine wave
  setUpNewFreq(nFreq);
  // Delay until level is constant
  delay(ZDELAY + (unsigned long)(1000.0 / FreqData[nFreq].freqHz));
  measureT();      // result is Tmeas
  T[nFreq] = Tmeas;
  
  LCDPrintSingleT(nFreq);
  }

// doTSweep() measures transmission at all sweep frequencies. The calling
// parameter, out sets Serial.print.  This is for serial controlled sweep.
void doTSweep(bool out)
  {
#if 0
  // Check that cal done, settings OK
  if (!calTSweep)
    {
    if(out)
       Serial.println("Sweep cannot run without Cal---use CAL or C");
    doRun = RUNNOT;
    return;
    }
#endif
  DC1.amplitude(dacLevel);     // Turn on sine wave   <<<<<<<<<MAKE PROGRAMMABLE
  for (nFreq = 1; nFreq <= 13;  nFreq++) // Over all frequencies; nFreq is global freq index
    {
    setUpNewFreq(nFreq);
    delay(ZDELAY + (unsigned long)(1000.0 / FreqData[nFreq].freqHz));
    measureT();      // result is Tmeas
    T[nFreq] = Tmeas;
    }
  }

//===========================  IMPEDANCE  ============================

// measureZ does a single impedance data point, including applying CAL.
// Freq needs to be setup before calling, but the frequency index is iF.
// No delays for settling. No print.  Complements measureT.
// Results are global Z[iF], Y[iF], sLC[iF], pLC[iF] and Q[iF]
void measureZ(uint16_t iF)
  {
  Complex Yin(0.0f, 0.0f);
  Complex Z22(0.0f, 0.0f);
  Complex Yi(0.0f, 0.0f);
  Complex Yinput(0.0f, 0.0f);
  Complex Zmeas(0.0f, 0.0f);
  Complex Ymeas(0.0f, 0.0f);
  Complex Vm(0.0f, 0.0f);
  Complex Vr(0.0f, 0.0f);
  float32_t w, ZM;

  w = 6.2831853f * FreqData[nFreq].freqHz;
  getFullDataPt();    // Gets amplitudes and phases
  checkOverload();
  // Do corrections for analog amplifier errors, vRatio and dPhase:
  Vm = polard2rect(amplitudeV * FreqData[nFreq].vRatio, phaseV + FreqData[nFreq].dPhase);
  Vr = polard2rect(amplitudeR, phaseR);
  // The next Complex(,) is the reference resistor value, set up to be changed and made sticky
  Zmeas = Complex(uSave.lastState.valueRRef[uSave.lastState.iRefR], 0.0) * Vm / (Vr - Vm);
  // Note Zmeas is re-used below

  // The input capacity and the input to U1D needs de-embedding.  This consists of 3 elements,
  // the 0.22 uF coupling cap, the 1.0 megohm shunt resistor and stray capacitance.
  // In parts:
  Z22 = Complex(0.0, -1.0) / (w*uSave.lastState.capCouple);  // Impedance of 0.22 coupling cap

  // A digression - For TUNEUP 4 command, we need the measured input Z plus the same corrected for the 0.22 uF
  // coupling cap.  This is global to get it to the TUNEUP calculation. Not used here further.
  Ztuneup0 = Zmeas;          // No correction for .22 cap
  Ztuneup  = Zmeas - Z22;
  
  // Yi is admittance of input R and C in parallel
  Yi = Complex((1.0 / uSave.lastState.resInput), 0.0) + Complex(0.0, w * uSave.lastState.capInput);
  Yinput = Cone / (Z22 + (Cone / Yi));                    // Admittance shunting input
  Ymeas = Cone / Zmeas;                                   // Uncorrected measured suseptance
  if(verboseData && useUSB && !doingNano)
     {
     Serial.println("");
     Serial.print("Embedded Z measured: "); Serial.print(Zmeas.real(),4);
             Serial.print("+j"); Serial.println(Zmeas.imag(),4);
     Serial.print("Embedded Y measured:");  Serial.print(Ymeas.real(), 9);
             Serial.print(" <re(Y)   im(Y)> "); Serial.print(Ymeas.imag(), 9);
             Serial.print("  pF > ");  Serial.println(1.0E12*Ymeas.imag() / w);
     }
  // Re-use Zmeas and Ymeas for final corrected values:
  Ymeas = Ymeas - Yinput;                      // De-embed correction to getunknown complex Y
  // And a final correction for resistance and inductive reactance of measuring leads, seriesR, seriesL.
  Zmeas = (Cone / Ymeas) - Complex(uSave.lastState.seriesR, w*uSave.lastState.seriesL);
  Ymeas = Cone / Zmeas;        // Include the correction back to Y
  // Indicate that a better refR may be available
  ZM = sqrt(Zmeas.real() * Zmeas.real() + Zmeas.imag() *Zmeas.imag());
  if(avnaState != WHATSIT)
     {
     if(uSave.lastState.iRefR == R50  &&  ZM > 500.0)
        LCDPrintError("Consider using refR=5K");
     else if(uSave.lastState.iRefR == R5K  &&  ZM < 500.0)
        LCDPrintError("Consider using refR=50");
     else
        LCDPrintError("");                   // Clear error
     }

  if( verboseData && useUSB && !doingNano )
     {
     Serial.print("De-embedded Z measured: "); Serial.print(Zmeas.real(),4);
             Serial.print("+j"); Serial.println(Zmeas.imag(),4);
     Serial.print("De-embedded Y measured: "); Serial.print(Ymeas.real(), 9);
             Serial.print(" <re(Y)   im(Y)> "); Serial.print(Ymeas.imag(), 9);
             Serial.print("  pF > ");  Serial.println(1.0E12*Ymeas.imag() / w);
     Serial.print("Measured: V="); Serial.print(amplitudeV);
             Serial.print(" VR="); Serial.print(amplitudeR);
             Serial.print(" Cal Ratio="); Serial.print(FreqData[nFreq].vRatio, 5);
             Serial.print(" dPhase="); Serial.println(FreqData[nFreq].dPhase, 3);
     }
  // Put results into global arrays, corrected for all strays
  Z[iF] = Zmeas;
  Y[iF] = Ymeas;
  if (Zmeas.imag() < 0.0)          // series R-C
     {
     sLC[iF] = -1.0 / (w * Zmeas.imag());
     // Zmeas.real() can be negative (usually very high Q with errors)
     if(Zmeas.real() > 0.0)
        Q[iF] = -Zmeas.imag() / Zmeas.real();     // Same Q, series or parallel
     else
        Q[iF] = 9999.9;
     }
  else                             // series R-L
     {
     sLC[iF] = Zmeas.imag() / w;
     if(Zmeas.real() > 0.0)
        Q[iF] = Zmeas.imag() / Zmeas.real();
     else
        Q[iF] = 9999.9;
     }

  if (Ymeas.imag() < 0.0)          // parallel R-L
     pLC[iF] = -1.0 / (w * Ymeas.imag());
  else                             // parallel R-C
     pLC[iF] = Ymeas.imag() / w;
// Data dump for nano testing
#if 0
     Serial.print("Measured: V="); Serial.print(amplitudeV);
             Serial.print(" VR="); Serial.print(amplitudeR);
             Serial.print(" Cal Ratio="); Serial.print(FreqData[nFreq].vRatio, 5);
             Serial.print(" dPhase="); Serial.println(FreqData[nFreq].dPhase, 3);
     Serial.print("Measured: R="); Serial.print(Zmeas.real(),5);
             Serial.print(" X="); Serial.println(Zmeas.imag(),5);
#endif
  }

// measureT does a single transmission data point, including applying CAL.
// Freq needs to be setup before calling, being found in FreqData[nFreq].freqHz.
// No printing is done.  No delays for settling.  Complements measureZ.
// Output is a single global Complex, Tmeas.  This is the ratio of the transmission
// transfer function to the reference value found by the thru-cal, with both
// adjusted for vRatio and dPhase.
void measureT(void)
  {
  float vGain, vPhase, vGainNorm, vPhaseNorm;
  //Complex Vm(0.0, 0.0);
  //Complex Vr(0.0, 0.0);

  getFullDataPt();
  checkOverload();
  // .vRatio for transmission is the through cal voltage magnitude.
  // .dPhase for transmission is the through cal voltage phase.
  vGain = (amplitudeV / amplitudeR) * FreqData[nFreq].vRatio;
  vPhase = phaseV - phaseR + FreqData[nFreq].dPhase;
  // Now normalize these gains to the through path gain from CAL.
  vGainNorm = vGain / FreqData[nFreq].thruRefAmpl;
  vPhaseNorm = vPhase - FreqData[nFreq].thruRefPhase;
  if (vPhaseNorm < (-180.0))
     vPhaseNorm += 360.0;
  else if (vPhaseNorm > 180.0)
     vPhaseNorm -= 360.0;
  Tmeas = polard2rect(vGainNorm, vPhaseNorm);
  if ((instrument = AVNA) && !doingNano)
     {
     Serial.print(FreqData[nFreq].freqHz, 3);
     if(uSave.lastState.tsData == 0)   // Print dB and phase
        {
        if (annotate)
           Serial.print(" Hz  Gain = ");
        else
           Serial.print(",");
        Serial.print(20.0*log10f(vGainNorm), 3);
        if (annotate)
           Serial.print(" dB  Phase (deg) = ");
        else
           Serial.print(",");
        Serial.println(vPhaseNorm, 3);
        }
     else     // Serial print ratio of voltages
        {
        if (annotate)
           Serial.print(" Hz  Av= ");
        else
           Serial.print(",");
        Serial.print(vGainNorm, 6);
        if (annotate)
           Serial.print(" V/V  Phase (deg)= ");
        else
           Serial.print(",");
        Serial.println(vPhaseNorm, 3);
        }
     }   // End if nanoState==DATA_NANO
  }

  /* getFullDataPt()  -  Measure average amplitude and phase (rel to DSP generated
   wave).  This function does not return until printReady istrue, meaning that the
   measurement is complete.  The frequency and ZorT must be set before calling here.
   Answers are global floats amplitudeV, phaseV, amplitudeR, phaseR.
   This function is useable for single or sweep, impedance or transmission.
*/
boolean getFullDataPt(void)
  {
  uint16_t kk;

  getNmeasQI();    // Measure and average   << Combine back here
  // The four data items will not become available in any particular order.
  // So track their status with NNReady[]
  // serial transmit data
  if (NNReady[0] && NNReady[1] && NNReady[2] && NNReady[3])    // Everybody ready
    {
    // Find ave power, correct for noise, and convert to an amplitude
    // The voltage at the top of the measurement resistor is measured
    // as ref Q, superAveNN[2], and ref I, superAveNN[3]. The superAveNN[] are doubles.
    amplitudeV = sqrtf( (float32_t) (superAveNN[1] * superAveNN[1] + superAveNN[0] * superAveNN[0]) );
    amplitudeR = sqrtf( (float32_t) (superAveNN[3] * superAveNN[3] + superAveNN[2] * superAveNN[2]) );
    // Minus signs reflect the hookup of the inputs to U1A and U1D:
    phaseV = r2df(atan2f(-(float32_t)superAveNN[0], -(float32_t)superAveNN[1]));
    phaseR = r2df(atan2f( (float32_t)superAveNN[2],  (float32_t)superAveNN[3]));
    // Diagnostic printout
    if(DIAGNOSTICS)
      {
      Serial.print("V values: "); Serial.print(amplitudeV); Serial.print("  ang in deg = "); Serial.println(phaseV);
      Serial.print("R values: "); Serial.print(amplitudeR); Serial.print("  ang in deg = "); Serial.println(phaseR);
      }
    for (kk = 0; kk < 4; kk++)
      NNReady[kk] = false;   // Only print these once
    printReady = true;
    return true;
    }
  return false;
  }

/* getNmeasQI(Nmeas, zDelay)
   Gather  numTenths x numCycles   samples of I, Q, RI and RQ and average.
   zdelay = the settling time in ms before starting.
   Results are normalized to one measurement.
   Results in double superAveNN[]
*/
void getNmeasQI(void)
  {
  uint16_t hh, jj, kk;

  for (jj = 0; jj < 4; jj++)
     {
     queueNN[jj].begin();    // Start loading the queueNN[]
     queueNN[jj].clear();    //  Clear everything
     sumNN[jj] = 0.0f;        // 4 places for results
     }
   countMeasurements = 0;  // Total number of measurements, like up to 4411 for 0.1 sec.
   for (hh = 0; hh < FreqData[nFreq].numTenths; hh++)  // All tenth seconds    <<<<<
     {
     // Get most of data for measurement
     for (kk = 0; kk < num256blocks; kk++)
        {
        // Get data, after all queues are loaded
        while ((queueNN[0].available() < 2) || (queueNN[1].available() < 2) ||
               (queueNN[2].available() < 2) || (queueNN[3].available() < 2));   // Just wait
        // All are available;  Go get 'em
        countMeasurements += 256;
        for (jj = 0; jj < 4; jj++)
           {
           superAveNN[jj] += (double)get2blockAve(jj, 256);  // Righht side is float
           }
        }            // End, over all full blocks
    // In general a partial block of less than 256 is still needed
    while ((queueNN[0].available() < 2) || (queueNN[1].available() < 2) ||
           (queueNN[2].available() < 2) || (queueNN[3].available() < 2));   // Just wait
    // numCycles of them are available;  Go get 'em
    countMeasurements += numCycles;
    for (jj = 0; jj < 4; jj++)
       {
       superAveNN[jj] += (double)get2blockAve(jj, numCycles);
       }
     }      // End, over all tenth seconds

  // Divide by countMeasurements to get average
  for (jj = 0; jj < 4; jj++)
     {
     queueNN[jj].end();     //  Stop queues
     superAveNN[jj] = superAveNN[jj] / ((double)countMeasurements);
     NNReady[jj] = true;     // Mark as ready to use and print
     }
  }

/* get2blocks(uint16_t nn)  The basic measurements are averaged here.
   nn is the meauremnt 0 to 3 (Q, I, RQ, RI).
   The average, a float, is returned in sumNN[nn]
   Summing 8-bits worth of 16-bit numbers, to a 23-bit float.  Should work
*/
float32_t get2blockAve(uint16_t nn, uint16_t npts)
  {
  uint16_t ii;

  // Fetch 2 blocks from nn'th of 4 queues and copy
  // into a 512 byte (256 16-bit word) buffer.
  // The return is the sum of up to 256 and is left in global float sumNN[nn]
  // memcpy() works on bytes, bufferNN[] works on 16-bit words
  memcpy(&bufferNN[nn][0], queueNN[nn].readBuffer(), 256);
  queueNN[nn].freeBuffer();
  memcpy(&bufferNN[nn][128], queueNN[nn].readBuffer(), 256);
  queueNN[nn].freeBuffer();
  sumNN[nn] = 0.0f;
  // Add npts samples (16-bit numbers) to sumNN[nn]
  for (ii = 0; ii < npts; ii++)
     {
     sumNN[nn] += (float32_t)bufferNN[nn][ii];
     }
  return sumNN[nn];
  }

// Someday:  Provide a field in non-annotated output for warnings and errors  <<<<<<<
void checkOverload(void)
  {
  float ppM, ppR;
  if (annotate && pkDet.available() && (ppM = pkDet.readPeakToPeak()) > 1.98)
    {
    Serial.print("Warning: P-P from Measure ADC=");
    Serial.print(50.0 * ppM);
    Serial.println("%");
    }
  if (annotate && pkDetR.available() && (ppR = pkDetR.readPeakToPeak()) > 1.98)
    {
    Serial.print("Warning: P-P from Reference ADC=");
    Serial.print(50.0 * ppR);
    Serial.println("%");
    }
  }

// Utility to convert complex vector in amplitude and phase in
// degrees into conventional rectangular complex vector.
Complex polard2rect(float32_t a, float32_t p)
  {
  return Complex( a*cosf(d2rf(p)), a*sinf(d2rf(p)) );
  }
