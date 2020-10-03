
// Display 7 lines of impedance data on LCD
void display7Z(void)
  {
  uint16_t i, nfr;
  float32_t XX, RR, ZM;
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(20, 38);
  tft.print("Freq        R    +j    X           L-C");
  if(dataValidZSweep)
     for(i=0; i<7; i++)
        {
        nfr = i + nFreqDisplay;
        RR = Z[nfr].real();
        XX = Z[nfr].imag();
        ZM = sqrtf(RR*RR + XX*XX);
        tft.setCursor(0, 57+19*i);
        if     (FreqData[nfr].freqHz < 100.0f)
           tft.print(FreqData[nfr].freqHz, 2);
        else if(FreqData[nfr].freqHz < 1000.0f)
           tft.print(FreqData[nfr].freqHz, 1);
        else    //  Must be 1000 or more
           tft.print(floor(0.5+FreqData[nfr].freqHz), 0);
        tft.print("Hz");
        tft.setCursor(70, 57+19*i);
        if(fabs(RR)<10.0f)
           tft.print(RR, 3);
        else if(fabsf(RR)<100.0f)
           tft.print(RR, 2);
        else if(fabsf(RR)<1000.0f)
           tft.print(RR, 1);
        else
           tft.print(RR, 0);
        tft.setCursor(155, 57+19*i);
        if(fabs(XX)<10.0f)
           tft.print(XX, 3);
        else if(fabs(XX) < 100.0f)
           tft.print(XX, 2);
        else if(fabs(XX) <1000.0f)
           tft.print(XX, 1);
        else
           tft.print(XX, 0);
        tft.setCursor(220, 57+19*i);
        if(XX<=0.0 && (Q[nfr]>1.0f || Q[nfr]<0.0f))          // Call it a capacitor
           {
           tft.print(valueString(sLC[nfr], cUnits));
           }
        else if(XX>0.0 && (Q[nfr]>1.0f || Q[nfr]<0.0f))      // Call it an inductor
           {
           tft.print(valueString(sLC[nfr], lUnits));
           }
        else     // Low Q, show as a resistor with series L or C
           {
           if(XX > 0.0f)
              {
              tft.print(valueString(sLC[nfr], lUnits));
              }
           else
              {
              tft.print(valueString(sLC[nfr], cUnits));
              }
           }
        if(uSave.lastState.iRefR == R5K)
            printQuality(ZM / 5000.0f);
        else
            printQuality(ZM / 50.0f);
        }
  }

// Display 7 lines of transmission data on LCD
void display7T(void)
  {
  uint16_t i, nfr;
  float32_t XT, RT, MT, PT, MTdB;
  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  if(dataValidTSweep)
     for(i=0; i<7; i++)
        {
        nfr = i + nFreqDisplay;
        RT = T[nfr].real();
        XT = T[nfr].imag();
        MT = sqrtf(RT*RT + XT*XT);
        MTdB = 20.0f * log10f(MT);
        PT = r2df(atan2f(XT, RT));
        tft.setCursor(10, 43+19*i);
        tft.print(FreqData[nfr].freqHz, 0);   tft.print(" Hz");
        tft.setCursor(95, 43+19*i);
        tft.print("G=");  tft.print(MTdB);  tft.print("dB");
        tft.setCursor(190, 43+19*i);
        tft.print("Ph=");
        tft.print(PT);  tft.print("Deg");
        }
  }

void displaySweepRange(void)
  {
  tft.fillRect(0, 140, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(20, 141);
  tft.print("Display ");   tft.print(FreqData[nFreqDisplay].freqHz, 0);   tft.print(" to ");
  tft.print(FreqData[6 + nFreqDisplay].freqHz, 0);   tft.print(" Hz");
  }

/* LCDPrintSingleZ()  -  Produces a screen based on data in globals
 * Z[0], Y[0], etc.
 * useLCD is checked before calling this fcn
 */
void LCDPrintSingleZ(uint16_t nfr)
  {
  float32_t XX, RR, GG, BB;

  RR = Z[nfr].real();
  XX = Z[nfr].imag();
  GG = Y[nfr].real();
  BB = Y[nfr].imag();

  if(useLCD)         // Print screen of single frequency data
    {
   tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_14);
    tft.setCursor(20, 38);
    /*
    tft.print("Z - Freq-");
    if     (FreqData[nfr].freqHz < 100.0)
       tft.print(FreqData[nfr].freqHz, 2);
    else if(FreqData[nfr].freqHz < 1000.0)
       tft.print(FreqData[nfr].freqHz, 1);
    else    //  Must be 1000 or more
       tft.print(floor(0.5+FreqData[nfr].freqHz), 0);
    tft.print(" Hz  Ref=");
    if (uSave.lastState.iRefR==R50)
       tft.print("50.0 Ohm");
    else
       tft.print("5.0 KOhm");
    */

#if 0
    //This doesn't seem to be useful ???
    tft.print("RL=");
    tft.print(RetLoss, 2);
    tft.print("dB  Phase=");
    tft.print(ReflPhase);
    tft.print("deg");
#endif

    tft.setCursor(20, 67);
    tft.print("Series: R=");
    tft.print(valueStringSign(RR, rUnits));
    tft.print("  X=");
    tft.print(valueStringSign(XX, rUnits));
    tft.setFont(Arial_18);
    tft.setCursor(30, 85);
    if(Q[nfr]>1.0f && XX<0.0f)
       {
       tft.print("C=");
       tft.print(valueString(sLC[nfr], cUnits));
       tft.print("  Q=");
       // Serial.print shows negative Q's as a number, +9999.9.  Here we can be less terse:
       if(Q[nfr] < 9999.8f)
          tft.print(Q[nfr], 1);
       else
          tft.print(" Neg");
       }
    else if(Q[nfr]>1.0f && XX>=0.0f)
       {
       tft.print("L=");
       tft.print(valueString(sLC[nfr], lUnits));
       tft.print("  Q=");
       if(Q[nfr] < 9999.8f)
          tft.print(Q[nfr], 1);
       else
          tft.print(" Neg");
       }
    else     // Low Q, show as a resistor with series L or C
       {
       tft.print("R=");
       tft.print(valueStringSign(RR, rUnits));
       if(XX > 0.0f)
          {
          tft.print(" L=");
          tft.print(valueString(sLC[nfr], lUnits));
          }
       else
          {
          tft.print(" C=");
          tft.print(valueString(sLC[nfr], cUnits));
          }
        }
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_14);
    tft.setCursor(20, 129);
    // G and B are the correct wasy to express a complex admittance.
    // parallelGB = false causes the inverted values of parallal R and X to be shown
    if(parallelGB)
       {
       tft.print("Par.: G=");
       tft.print(valueStringSign(GG, gUnits));
       tft.print("  B=");
       tft.print(valueStringSign(BB, gUnits));
       }
    else
       {
       tft.print("Par.: R=");
       if(fabsf(GG) > 1.0E-15)                        // What is #define for min??
          tft.print(valueStringSign(1.0f/GG, rUnits));  // Equiv parallel res
       else
          tft.print("  inf   ");
       tft.print("  X=");
       if(fabsf(BB) > 1.0E-15)
          tft.print(valueStringSign(1.0f/BB, rUnits));
       else
          tft.print("  inf   ");
       }
    tft.setFont(Arial_18);
    tft.setCursor(30, 151);
    if(Q[nfr]>1.0f && BB>=0.0f)
       {
       tft.print("C=");
       tft.print(valueString(pLC[nfr], cUnits));
       tft.print("  Q=");
       if(Q[nfr] < 9999.8f)
          tft.print(Q[nfr], 1);
       else
          tft.print(" Neg");
       }
    else if(Q[nfr]>1.0f && BB<0.0f)
       {
       tft.print("L=");
       tft.print(valueString(pLC[nfr], lUnits));
       tft.print("  Q=");
       if(Q[nfr] < 9999.8f)
          tft.print(Q[nfr], 1);
       else
          tft.print(" Neg");
       }
    else
       {
       tft.print("R=");
       if(fabsf(GG) > 1.0E-37)                        // What is #define for min??
          tft.print(valueStringSign(1.0f/GG, rUnits));
       else
          tft.print("  inf   ");
       if(BB < 0.0f)
          {
          tft.print(" Lp=");
          tft.print(valueString(pLC[nfr], lUnits));
          }
       else
          {
          tft.print(" Cp=");
          tft.print(valueString(pLC[nfr], cUnits));
          }
        }
     }
// primaryControl XCTRL_LCD  XCTRL_USB
  }      // end LCDPrintSingle()

void LCDPrintError(char const *emsg)
  {
   tft.fillRect(130, 185, tft.width(), 14, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setFont(Arial_11);
    tft.setCursor(130,185);
    tft.print(emsg);
    tft.setTextColor(ILI9341_YELLOW);   // Leave it in yellow
  }


/* LCDPrintSweepLineZ()  -  Produces a sweep line based on data in globals
 * Z[], Y[], etc.
 * useLCD is checked before calling this fcn
 */
void LCDPrintSweepZ(void)
  {
  double XX, RR;
  uint16_t ii, nfr;

  tft.fillRect(0, 38, tft.width(), 161, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setFont(Arial_12);
  tft.setCursor(0, 38);
  tft.print(" FREQ      R        X          L/C");

  for(ii=0; ii<7; ii++)
    {
    nfr = ii+firstLCDSweepFreq;
    RR = Z[nfr].real();
    XX = Z[nfr].imag();
    // Print line of Z data
    tft.setCursor(0, 60+20*ii);
    if     (FreqData[nfr].freqHz < 100.0f)
       tft.print(FreqData[nfr].freqHz, 2);
    else if(FreqData[nfr].freqHz < 1000.0f)
       tft.print(FreqData[nfr].freqHz, 1);
    else    //  Must be 1000 or more
       tft.print(floor(0.5+FreqData[nfr].freqHz), 0);
    tft.print("  ");
    tft.print(valueStringSign(RR, rUnits));
    tft.print("  ");
    tft.print(valueStringSign(XX, rUnits));
    tft.print("  ");
    if(XX<0.0f)
       tft.print(valueString(sLC[nfr], cUnits));
    else if(XX>=0.0f)
       tft.print(valueString(sLC[nfr], lUnits));
    }
  }      // end LCDPrintSingle()


/* LCDPrintSingleT()  -  Produces a screen based on data in global
 * T[0]
 */
void LCDPrintSingleT(uint16_t nfr)
  {
  double XT, RT, MT, PT, MTdB;

  RT = T[nfr].real();
  XT = T[nfr].imag();
  MT = sqrt(RT*RT + XT*XT);
  MTdB = 20.0f * log10(MT);
  PT = r2df(atan2(XT, RT));

  if(useLCD)         // Print screen of single frequency data
    {
    tft.fillRect(0, 38, tft.width(), 146, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_13);
    tft.setCursor(20, 38);
    tft.print("Max Freq, kHz ");
    if     (FreqData[nfr].freqHz < 100.0f)
       tft.print(FreqData[nfr].freqHz, 2);
    else if(FreqData[nfr].freqHz < 1000.0f)
       tft.print(FreqData[nfr].freqHz, 1);
    else    //  Must be 1000 or more
       tft.print(floor(0.5+FreqData[nfr].freqHz), 0);
    tft.print(" Hz");

    tft.setCursor(20, 62);
    if (uSave.lastState.iRefR==R50)
       tft.print("Ref=50.0 Ohm");
    else
       tft.print("Ref=5.0 KOhm");

    tft.setCursor(20, 86);
    tft.print("Voltage Gain = ");
    tft.print(MT, 5);
    tft.print(" V/V");

    tft.setFont(Arial_18);
    tft.setCursor(30, 120);
    tft.print("Gain = ");
    tft.print(MTdB, 3);
    tft.print(" dB");

    tft.setCursor(30, 155);
    tft.print("Phase = ");
    tft.print(PT, 2);
    tft.print(" deg");
    }
  }

void topLines(void)
  {
  topLine1();
  topLine2();
  topLine2a();
  }

void topLine1(void)
  {
  tft.fillRect(0, 0, tft.width(), 22, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(Arial_16);
  tft.setCursor(60, 0);
  tft.print("Audio Test Instrument");
  if(SDCardAvailable)
    drawScreenSaveBox(ILI9341_GREEN);
  else
    drawScreenSaveBox(ILI9341_BLACK);
  tft.setFont(Arial_9);
  tft.setTextColor(ILI9341_YELLOW);      // Others expect this
  }

void topLine2(void)
  {
  tft.fillRect(0, 22, tft.width(), 16, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(5, 22);
  // ToDo: setup a function for this to allow changing settings
  tft.print("Ver 0.80 de W7PUA");
  tft.setTextColor(ILI9341_YELLOW);
  }

// topLine2a can be called only after topline2().
void topLine2a(void)
  {
  tft.setTextColor(ILI9341_WHITE);
  if(uSave.lastState.ZorT == IMPEDANCE)
    tft.print(" Z  f=");
  else
    tft.print("   f=");
  tft.print(FreqData[nFreq].freqHzActual, 3);
  tft.print("Hz    Ref=");
  tft.print(uSave.lastState.valueRRef[uSave.lastState.iRefR]);
  tft.setTextColor(ILI9341_YELLOW);
  }

/* *******  Save Screen to BMP file  *******
 * Following is from ScreenToSDbmp by Steven T. Stolen / K7CCR
 * See https://forum.pjrc.com/threads/42679-ILI9341-Screen-Dump-to-SD-Card-on-Teensy-3-6?highlight=Steve+Stolen
 * - Find next available name
 * - Open File
 * - Format file header records and write to SD file.
 * - Write rgb as 3 bytes for full screen, bottom to top
 * - Close the file.
 *
 * For the AVNA we provide 2 options 1-BMP file to SD Card (if card in place)
 *                                   2-HEX encoded BMP to USB-Serial (Screen serial command)
 * Global control:  bmpScreenSDCardRequest   Write BMP file to SD card
 *                  hexScreenRequest         Send BMP file over USB serial
 */
void dumpScreenToSD(void) {
  uint16_t colorPixel;
  uint8_t r, g, b;
  uint16_t x = 0;
  uint16_t y = 0;
  const uint16_t width = 320;
  const uint16_t height = 240;
  const uint16_t linesize = 3 * width;
  uint8_t linebuf[linesize];
  const uint32_t filesize = 54 + 3 * width * height;
  // readPixel() combines RGB to 16-bit 565 format, readRect gets many of these
  uint16_t pixelColorArray[width];
  char filename[] = "AVNA1_00.BMP";
  unsigned char bmpfileheader[14] = {'B','M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};

  #define BMP_DEBUG 0
  #define BMP_DEFINE 0

  /* xBITMAPFILEHEADER  -  for eeference info
   *
   * struct xBITMAPFILEHEADER
   * {
   * uint16_t bfType;  * signature - 'BM'
   * uint32_t bfSize; // file size in bytes
   * uint16_t bfReserved1; // 0
   * uint16_t bfReserved2; // 0
   * uint32_t bfOffBits; // offset to bitmap
   * };
   *
   * union fh_data {
   * struct xBITMAPFILEHEADER fh;
   * uint8_t fhData[sizeof(xBITMAPFILEHEADER)];
   * } fhFrame;
   *
   * union fhKludge {
   * uint32_t bfSize;
   * uint8_t kludge[4];
   * } kludgeFrame;
   */
  struct xBITMAPINFOHEADER
  {
  uint32_t biSize;         // size of this struct
  int32_t  biWidth;        // bmap width in pixels
  int32_t  biHeight;       // bmap height in pixels
  uint16_t biPlanes;       // numplanes - always 1
  uint16_t biBitCount;     // bits per pixel
  uint32_t biCompression;  // compression flag
  uint32_t biSizeImage;    // image size in bytes
  int32_t  biXPelsPerMeter; // horz resolution
  int32_t  biYPelsPerMeter; // vert resolution
  uint32_t biClrUsed;      // 0 -> color table size
  uint32_t biClrImportant; // important color count
  };

  union ih_data {
  struct xBITMAPINFOHEADER ih;
  uint8_t ihData[sizeof(xBITMAPINFOHEADER)];
  } ihFrame;

  // Format BMP File Name - determine if unique (so we can open)
  // Open File
  for (uint8_t i=0; i<100; i++)
    {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (!SD.exists(filename))
      {
      // Only open a new file if it does not exist
      bmpFile = SD.open (filename, FILE_WRITE);
      break;
      }
    }
  if (bmpFile)
    SDCardAvailable = true;
  else
    {
    if(verboseData)
      {
      Serial.print ("Could not create file: ");
      Serial.println (filename);
      }
    }
  if(verboseData)
    {
    Serial.print ("Opened new file: ");
    Serial.println(filename);
    }

  //Format file header and write
  bmpfileheader[2] = (filesize);
  bmpfileheader[3] = (filesize>>8);
  bmpfileheader[4] = (filesize>>16);
  bmpfileheader[5] = (filesize>>24);
  if(SDCardAvailable && bmpScreenSDCardRequest)
    bmpFile.write(bmpfileheader, sizeof(bmpfileheader));
  if(hexScreenRequest)
    {
    Serial.println("-----   Start BMP in HEX   -----");
    for (int k=0; k<14; k++) {
      Serial.print(bmpfileheader[k] < 16 ? "0" : "");
      Serial.print(bmpfileheader[k], HEX);
      Serial.print(" ");
      }
    }
  //Format info header and write
  ihFrame.ih.biSize = sizeof(xBITMAPINFOHEADER);
  ihFrame.ih.biWidth = width;
  ihFrame.ih.biHeight = height;
  ihFrame.ih.biPlanes = (uint16_t) 1;
  ihFrame.ih.biBitCount = (uint16_t) 24;
  ihFrame.ih.biCompression = (uint32_t) 0;
  ihFrame.ih.biSizeImage = width * height;
  ihFrame.ih.biXPelsPerMeter = (uint32_t) 0;
  ihFrame.ih.biYPelsPerMeter = (uint32_t) 0;
  ihFrame.ih.biClrUsed = (uint32_t) 0;
  ihFrame.ih.biClrImportant = (uint32_t) 0;
  for (uint8_t i=0; i<40; i++)    // Copy the info header to linebuf
    linebuf[i] = ihFrame.ihData[i];

  if(SDCardAvailable && bmpScreenSDCardRequest)
    bmpFile.write(linebuf, 40);
  if(hexScreenRequest)
    {
    for (int k=0; k<40; k++)
      {
      Serial.print(ihFrame.ihData[k] < 16 ? "0" : ""); // leading zero
      Serial.print(ihFrame.ihData[k], HEX);
      Serial.print(" ");
      }
    Serial.println("");  // Adds a lf into hex file after header
    }

#if BMP_DEBUG
  //Dump the infoheader to sys mon
  Serial.println("\nxBITMAPINFOHEADER, 40 bytes: ");
  for (uint8_t i=0; i<40; i++) {
    Serial.print(linebuf[i] < 16 ? "0" : "");
    Serial.print(linebuf[i], HEX);
    Serial.print(" ");
    if((i+1)%8==0)
      Serial.println(" "); //After 8 byte dump,
  }
  Serial.println("Done printing header dumps.");
#endif
  // memset(linebuf,0,40); //clean up buffer

  // Pixel dump to file here
  delay(100);
  // Build BMP file records from display & write to file
  for(int16_t i = height-1; i >= 0; i--) {   // Bottom to top
    // Here we write the pixels to the SD. We fetch the pixel, trnslate
    // to rgb, stuff line buf with the colors swapped to b,g,r order.
    // Fetch whole line - rectangle height 1 pixel
    // tft.readRect(0, i, width, 1, pixelColorArray);  // Sometimes fails, slips colors G->R->B
    for (int16_t j=0; j<width; j++)
      {
      pixelColorArray[j] = tft.readPixel(j, i);  // So far, this has been reliable
      color565toRGB_1(pixelColorArray[j],r,g,b);
      linebuf[j*3] = b;
      linebuf[j*3 + 1] = g;
      linebuf[j*3 + 2] = r;
#if BMP_DEBUG
      if(i==238){       //(j==0 && i==60) {
        Serial.print("colorPixel at x=");
        Serial.print(j);
        Serial.print("   y=");
        Serial.print(i);
        Serial.print("  is 0X");
        Serial.println(pixelColorArray[j], HEX);

        Serial.print("Red = 0x");
        Serial.print(r, HEX);
        Serial.print(" in decimal: ");
        Serial.println(r, DEC);

        Serial.print("Grn = 0x");
        Serial.print(g, HEX);
        Serial.print(" in decimal: ");
        Serial.println(g, DEC);

        Serial.print("Blu = 0x");
        Serial.print(b, HEX);
        Serial.print(" in decimal: ");
        Serial.println(b, DEC);
        Serial.println(" ");
      }
#endif
    }  // End, over all pixels in line

    if(SDCardAvailable && bmpScreenSDCardRequest)
      {
      bmpFile.write(linebuf, linesize);  // Write the linebuf
      }
    if(hexScreenRequest)
      {
      for (int k=0; k<linesize; k++)
        {
        Serial.print(linebuf[k] < 16 ? "0" : "");
        Serial.print(linebuf[k], HEX);
        Serial.print(" ");
        }
      Serial.println("");  // This adds lf in hex, but makes it line for line
      }
  }    // End, over all lines

  if(hexScreenRequest)
    {
    Serial.println("-----   End BMP in HEX   -----");
    hexScreenRequest = false;
    }
  if(SDCardAvailable)
    {
    bmpFile.close();
    bmpScreenSDCardRequest = false;
    }
  }

// Utility function for getting card info
void  exploreSDCard(void)
  {
  uint32_t volumesize;
  // See if the card is present and can be initialized
  if (!SD.begin(chipSelect))
    {
    Serial.println("uSD Card not present (or failed).");
    return;
    }
  else
    {
    Serial.println("uSD Card present.");
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, chipSelect))
      {
      Serial.println("uSD card initialization failed.");
      return;
      }
    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card))
      {
      Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
      return;
      }
    else
      {
      Serial.print("Partition found: FAT");
      // print the type and size of the first FAT-type volume
      Serial.println(volume.fatType(), DEC);
      volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
      volumesize *= volume.clusterCount();       // we'll have a lot of clusters
      Serial.print("Volume size (Mbytes): ");
      Serial.println(volumesize/2048);

      Serial.println("Files found on the card (name, date and size in bytes): ");
      root.openRoot(volume);
      // list all files in the card with date and size
      root.ls(LS_R | LS_DATE | LS_SIZE);
      Serial.println("");
      }
    }
  }

  //color565toRGB_1   - converts 565 format 16 bit color to RGB
  // This  _1 version scales r, g, b (0,255); library ignores ls bits.
  static void color565toRGB_1(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
    uint16_t tc;
    tc = (color&0XF800)>>11;
    r = uint8_t(tc*8 + tc/4);

    tc = (color&0X07E0)>>5;
    g = uint8_t(tc*4 + tc/16);

    tc = color&0X001F;
    b = uint8_t(tc*8 + tc/4);

/*    r = (color>>8)&0x00F8;
    g = (color>>3)&0x00FC;
    b = (color<<3)&0x00F8;
*/
  }

#if 0
  //color565toRGB14   - converts 16 bit 565 format color to 14 bit RGB (2 bits clear for math and sign)
  //returns 00rrrrr000000000,00gggggg00000000,00bbbbb000000000
  //thus not overloading sign, and allowing up to double for additions for fixed point delta
  static void color565toRGB14_1(uint16_t color, int16_t &r, int16_t &g, int16_t &b) {
    r = (color>>2)&0x3E00;
    g = (color<<3)&0x3F00;
    b = (color<<9)&0x3E00;
  }

  //RGB14tocolor565   - converts 14 bit RGB back to 16 bit 565 format color
  static uint16_t RGB14tocolor565_1(int16_t r, int16_t g, int16_t b)
  {
    return (((r & 0x3E00) << 2) | ((g & 0x3F00) >>3) | ((b & 0x3E00) >> 9));
  }
#endif
