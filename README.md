# AudioTestInstrument
Follow on from W7PUA Audio Vector Network Analyzer adding other instruments.

WORK N PROGRESS -  Beta test level for rev 0.81.  Bob  W7PUA

### Version 0.80 & 0.81 Change Summary

All capabilities of Version 0.70 plus the following:

* Audio Spectrum Analyzer covering up to 40kHz with graphical display.
* Vector Voltmeter with frequency selectivity and adjustable phase offset.
* Three Signal Generators (added together)  with calibrated output.
* Gaussian White Noise Generator, add to three signal generators.
* Screen Save to BMP file for all functions.
* Calibration of input and output levels and for the touch screen.
* SINAD and S/N measures as part of Spectrum Analyzer
* Correction of several bugs, such as the handling of double-precision constants.
* Inclusion of external third-party libraries with this download for easy install.

![Control Screens, AVNA Test Instrument](/images/AVNA_2Scr1.gif)

![Signal Generator Screens, AVNA Test Instrument](/images/AVNA_2Scr2.gif)

![Vector Voltmeter and Spectrum Analyzer Screens, AVNA Test Instrument](/images/AVNA_2Scr3.gif)

### Summary Install Procedure
The details 
1. Install scurrent Arduino per https://www.arduino.cc/en/main/software
2. Install Teensyduino per https://www.pjrc.com/teensy/td_download.html
3. From github download AudioTestInstrument-master.zip
4. Un-Zip that file, in place.
5. The resulting single directory is AudioTestInstrument-master. Open it.
6. Move the directory AVNA8main to your personal Arduino directory.
7. Go to your Arduino IDE and menu File/Open the file AVNA8main/AVNA8main.ino.
8. In the Arduino IDE menu Tools/Board select Teensy 3.6.
9. In the Arduino menu Tools/Port select one with Teensy 3.6 in its name!
10. From the Arduino IDE select menu Sketch/Upload.

That should leave you with your AVNA loaded with the new version 0.81 program!

### Libraries 
The libraries required for the AVNA, in part come from external sources.
These have been included with the ZIP (or clone) download in the "src" directory.
The suffix R2 has been added to those that I did not write to avoid naming problems with
other libraries that may be in your PC.  These libraries are complexR2,
SerialCommandR2, synth_GaussianWhiteNoiseR2, CircularBufferR2, 
and analyze_fft1024_p.  Do not try to bring these in, they are part of the Zip.

There are other libraries that come as part of Teensyduino.  These need no
special action to install:  SD, Audio, SPI, SerialFlash, Wire, EEPROM,
XPT2046_Touchscreen, ILI9341_t3.
