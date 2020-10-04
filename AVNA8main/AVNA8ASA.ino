//  AVNA8ASA.ino
// Routines associated with The Spectrum Analyzer part of the
// AVNA audio vector analyzer. 
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
void doFFT(void)
  {
  float32_t specDB, maxOut, pTemp;
  static uint16_t countMax = 100;  //  Make global and user adjustable
  static uint16_t countAve = 0;
  float32_t aveFactorDB;
  static float32_t avePower[512];
  uint16_t iiMax;
  float32_t vm, vc, vp, R;

  if (fft1024p.available())
    {
    aveFactorDB = 10.0f*log10f(countMax);
    specMax = -200.0f;
    for (int ii=0; ii<512; ii++)
      {
      avePower[ii] += fft1024p.read(ii);   // Power average, 0.0 for sine wave full scale down to -80 or so
      }

    if(++countAve == countMax)  // Averaging is compete, get info and plot
      {
      specMax = 0.0f;
      for(int ii=0; ii<512; ii++)
        {
        if (avePower[ii] > specMax)  // Find highest peak of 512
          {
          specMax = avePower[ii];
          iiMax = ii;
          }
        }

      // Power at peak for top left corner, use 3 bins
      if(iiMax < 2)
         pwr10 = avePower[0] + avePower[1];
      else if(iiMax>509)
         pwr10 = avePower[510] + avePower[511];
      else
         pwr10 = avePower[iiMax] + avePower[iiMax + 1] + avePower[iiMax - 1];
      pwr10DB = 10*log10f(pwr10) - aveFactorDB;
      // Serial.print(iiMax-1);  Serial.print(" <iiMax-1    avePower[iiMax]> ");  Serial.println(avePower[iiMax-1]/(float)countMax, 5);
      // Serial.print(iiMax);    Serial.print(" <iiMax      avePower[iiMax]> ");  Serial.println(avePower[iiMax]  /(float)countMax, 5);
      // Serial.print(iiMax+1);  Serial.print(" <iiMax+1    avePower[iiMax]> ");  Serial.println(avePower[iiMax+1]/(float)countMax, 5);

      // Interpolate peak frequency for top right corner per DerekR/Granke procedure
      // Following from DerekR
      // https://forum.pjrc.com/threads/36358-A-New-Accurate-FFT-Interpolator-for-Frequency-Estimation
      // " 1) A record of length 1024 samples is windowed with a Hanning window
      //   2) The magnitude spectrum is computed from the FFT, and the two (adjacent)
      //      largest amplitude lines are found.  Let the largest be line L, and the
      //      other be either L+1, of L-1.
      //   3) Compute the ratio R of the amplitude of the two largest lines.
      //   4) If the amplitude of L+1 is greater than L-1 then
      //              f = (L + (2-R)/(1+R))*f_sample/1024
      //        otherwise
      //              f = (L - (2-R)/(1+R))*f_sample/1024  "
      if (iiMax > 0)
        vm = sqrtf(avePower[iiMax - 1]);
      vc = sqrtf(avePower[iiMax]);
      if (iiMax < 511)
        vp = sqrtf(avePower[iiMax + 1]);
      if(iiMax<2) 
        specMaxFreq = 0.0f;  // DC term is problematic.  Reduce sample rate for low frequencies.
      else if(vp > vm)
        {
        R = vc/vp;
        // 0.05 Hz upward error makes display more readable
        specMaxFreq = 0.05 + ( (float32_t)iiMax + (2-R)/(1+R) )*freqASA[ASAI2SFreqIndex].sampleRate/1024.0f;
        }
      else if(iiMax==511 || vp < vm)
        {
        R = vc/vm;
        specMaxFreq = 0.05 + ( (float32_t)iiMax - (2-R)/(1+R) )*freqASA[ASAI2SFreqIndex].sampleRate/1024.0f;
        }

      // Find x-y  for spectral plot
      for(int ii=0; ii<255; ii++)
        {
        // Combine 2 bins for each pixel, convert to dB
        pTemp = avePower[2*ii] + avePower[2*ii+1];

        if (pTemp > 0.0f)   // Don't log zero
          specDB = uSave.lastState.SAcalCorrectionDB + 10.0f*log10f(pTemp) - aveFactorDB;
        else
          specDB = -90.288f;  // 6 dB below lsb, arbitrary

        // pixelnew[]   0=bottom of display, 160=top.
        pixelnew[ii] = (int16_t)(160.0f+20.0f*(ASAdbOffset+specDB)/dbPerDiv);
        if (pixelnew[ii] < 0)
           pixelnew[ii] = 0;
        }   // End, over all 256 pixels
      for(int ii=0; ii<512; ii++)    // Clear powers for next measurement
        avePower[ii] = 0.0f;
      show_spectrum();
      countAve = 0;
      }  // End, if averging is finished
    }  // End, if fft available
  }

void prepSpectralDisplay(void) 
  {
  //Annotate the y-axis
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(Arial_10);
  tft.setCursor(95, 3);
  tft.print("Audio Spectrum Analyzer");
  for(int ii = 0; ii<5; ii++)
     {
     tft.setCursor(8, spectrum_y-4+40*ii);
     tft.print(-ASAdbOffset-2*ii*(int)dbPerDiv, 0);
     }
  tft.setCursor(10 , spectrum_y+16);
  tft.print("dBm");

  // Anotate the x-axis
  for(int ii = 0; ii<6; ii++)
     {
     if (ASAI2SFreqIndex > 0)
       {
       tft.setCursor(37+43*ii, 183);
       tft.print(ii*(int)(0.2*freqASA[ASAI2SFreqIndex].maxFreq));
       }
     else
       {
       tft.setCursor(37+43*ii, 183);
       tft.print((0.2*freqASA[ASAI2SFreqIndex].maxFreq)*(float)ii, 1);
       }
     }
   tft.setCursor(275, 183);
     tft.print("kHz");
  }

// Spectrum display based on DD4WH Convolution Radio
void show_spectrum()
  {
  int16_t y_old, y_new, y1_new, y1_old;
  int16_t y1_old_minus = 0;
  int16_t y1_new_minus = 0;

  if (instrument != ASA) return;

  // clear spectrum display
  tft.fillRect(spectrum_x-5, spectrum_y-1, 262, spectrum_height+2, ILI9341_BLACK);
  // prepare_spectrum_display();
  for(int ii=0; ii<=160; ii+=40)
      tft.drawFastHLine (spectrum_x-5, spectrum_y+ii,  259, ILI9341_ORANGE);
  for(int ii=20; ii<=160; ii+=40)
      tft.drawFastHLine (spectrum_x, spectrum_y+ii,  254, ILI9341_ORANGE);
  for(float jf=0.0f; jf<=241.0f; jf+=42.66667f)
      tft.drawFastVLine (spectrum_x+(int)(0.5+jf), spectrum_y, 165, ILI9341_ORANGE);
  for(float jf=21.33333f; jf<241.0f; jf+=42.66667f)
      tft.drawFastVLine (spectrum_x+int(0.5+jf), spectrum_y, 160, ILI9341_ORANGE);
  tft.drawFastVLine     (spectrum_x+255,  spectrum_y, 160, ILI9341_ORANGE);
  tft.drawFastVLine (spectrum_x+43, spectrum_y+2, 16, ILI9341_BLACK);  // Data text areas
  tft.drawFastVLine (spectrum_x+235, spectrum_y+2, 16, ILI9341_BLACK);

  // Draw spectrum display
  for (int16_t j = 0; j < 254; j++)
    {
    if ((j > 1) && (j < 255))
      {
      if (spectrum_mov_average)  // From DD4WH, not used for now
        {
        // moving window - weighted average of 5 points of the spectrum to smooth spectrum in the frequency domain
        // weights:  j: 50% , j-1/j+1: 36%, j+2/j-2: 14%
        y_new = pixelnew[j] * 0.5 + pixelnew[j - 1] * 0.18 + pixelnew[j + 1] * 0.18 + pixelnew[j - 2] * 0.07 + pixelnew[j + 2] * 0.07;
        y_old = pixelold[j] * 0.5 + pixelold[j - 1] * 0.18 + pixelold[j + 1] * 0.18 + pixelold[j - 2] * 0.07 + pixelold[j + 2] * 0.07;
        }
      else   // not spectrum_mov_average
        {
        y_new = pixelnew[j];
        y_old = pixelold[j];
        }
      }
    else    // x at edges, i.e., x not between 2 and 254
      {
      y_new = pixelnew[j];
      y_old = pixelold[j];
      }
    if (y_old > (spectrum_height + 1))
       y_old = (spectrum_height + 1);
    if (y_new > (spectrum_height + 1))
       y_new = (spectrum_height + 1);
    // Bob - See if we might have worked with the screen direction back in doFFT()  <<<<<<<<<<<<<<
    y1_old  = (spectrum_y + spectrum_height - 1) - y_old;
    y1_new  = (spectrum_y + spectrum_height - 1) - y_new;
    if (j == 0)
      {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
      }
    if (j == 254)
      {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
      }
    {
      // DELETE OLD LINE/POINT
      if (y1_old - y1_old_minus > 1)
        { // plot line upwards
        tft.drawFastVLine(j + spectrum_x, y1_old_minus + 1, y1_old - y1_old_minus, ILI9341_BLACK);
        }
      else if (y1_old - y1_old_minus < -1)
        { // plot line downwards
        tft.drawFastVLine(j + spectrum_x, y1_old, y1_old_minus - y1_old, ILI9341_BLACK);
        }
      else
        {
        tft.drawPixel(j + spectrum_x, y1_old, ILI9341_BLACK); // delete old pixel
        }
      // DRAW NEW LINE/POINT
      if (y1_new - y1_new_minus > 1)
        { // plot line upwards
        tft.drawFastVLine(j + spectrum_x, y1_new_minus + 1, y1_new - y1_new_minus, ILI9341_WHITE);
        }
      else if (y1_new - y1_new_minus < -1)
        { // plot line downwards
        tft.drawFastVLine(j + spectrum_x, y1_new, y1_new_minus - y1_new, ILI9341_WHITE);
        }
      else
        {
        tft.drawPixel(j + spectrum_x, y1_new, ILI9341_WHITE); // write new pixel
        }
      y1_new_minus = y1_new;
      y1_old_minus = y1_old;
      }
    } // End for(...) Draw 254 spectral points

    //tft.drawFastVLine( , , , ILI9341_BLACK);  Cursor point (addlater)
    //tft.drawFastVLine( , , , ILI9341_RED);
    tft.setCursor(70, 21);    // (48, 21);
    tft.print(uSave.lastState.SAcalCorrectionDB + pwr10DB, 2);
    tft.setCursor(260, 21);
    if(specMaxFreq>0.0f)
      tft.print(specMaxFreq, 1);
    else
      tft.print(" ---");
    for(int ii=0; ii<256; ii++)
        pixelold[ii] = pixelnew[ii];
  } // End show_spectrum()

