/*
 *  synth_GaussianWhiteNoise.h
 *  W7PUA     4 Oct 2020
 *
 * Copyright (c) 2020 Robert Larkin
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
 
/* This is a Gaussian White Noise generator that is compatible as an Audio
 * object for the Teensy 3.x or 4.x processors.  This produces 16-bit signed
 * integer outputs.  The statistical mean of the outputs is 0.0 and the standard
 * deviation (SD) is specified by amplitude() with a range of 0.0 to 1.0.  Since
 * the output is a signed integer, an SD of 1.0 would be -32768 to +32767.
 * This would produce a saturated full output 32% of the time.  So the SD
 * should be reduced to 0.333 for saturation at the "3-sigma" amount
 * of 0.03%, and so forth.  Default amplitude() is 0.0 (off).
 * 
 * There is no begin function, but when the AudioSynthGaussian object is created,
 * there is no Gaussian Noise output.  Use the amplitude(sd) function to
 * set the level.  Likewise, the LP Filter is not running at startup.
 * This is enabled by setting a frequency for the low pass cutoff, as
 * for exmple, "gwngen1.setLowPass(4200.0);". To turn the low pass filter back off,
 * set the low pass frequency to a value close to half the
 * sample frequency (within 95%).  For instance, for a 44.1 kHz sample rate, use
 * something like "gwngen1.setLowPass(22000.0);".  The latter settings prevents
 * the low pass filter code from executing, saving real time.
 *
 * Sample Program:  gaussianNoiseAndSignal.ino
 *
 *  Time requirements (these were measured  for the float, no LPF version and
 *  the fixed point here should take slightly more time):
 *   For generating a block of 128, Teensy 3.6, 121 microseconds
 *   For generating a block of 128, Teensy 4.0,  36 microseconds
 *
 * CREDITS:  Thanks to PJRC and Paul Stoffregen for the Teensy processor,
 * Teensyduino  and Teensy Audio.  Thanks to Chip Audette for the
 * F32 extension work.  All of that makes this possible. As an algorithm,
 * the "Even Quicker" uniform random sample generator from D. E. Knuth and
 * H. W. Lewis and described in Chapter 7 of "Numerical Receipes in C",
 * 2nd ed, with the comment "this is about as good as any 32-bit linear
 * congruential generator, entirely adequate for many uses."
 * The conversion from uniform to Gaussian distributions is done by adding
 * 12 random deviates (love that term!) together using the Central Limit
 * Theorem.  Further refs:
 * Park-Miller-Carta Pseudo-Random Number Generator
 * http://www.firstpr.com.au/dsp/rand31/
 */

#ifndef synth_GaussianNoise_h_
#define synth_GaussianNoise_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "utility/dspinst.h"

#define FL_ONE  0X3F800000
#define FL_MASK 0X007FFFFF

class AudioSynthNoiseGaussian : public AudioStream
{
public:
    AudioSynthNoiseGaussian() : AudioStream(0, NULL) {
        idum = 1357246891;
        sd =0.0f;
    }
    
    // Gaussian amplitude is specified by the 1-sigma (standard deviation) value.
    // sd=0.0 is un-enabled.
    void amplitude(float _sd) {
        sd = _sd;  // Enduring copy
        if (sd<0.0)  sd=0.0;
    }

    // This is for FUTURE.  It computes the IIR LPF if called, but NOT USED in update.
    // IIR BiQuad coefficients, See Teensy Audio Library and
    // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
    void setLowPass(float frequency) {
        double w0 = (double)frequency * 6.28318530718 / AUDIO_SAMPLE_RATE_EXACT;
        double alpha = sin(w0) * 0.7071067811865476;  // for Butterworth
        double cosW0 = cos(w0);
        double scale = 1.0 / (1.0 + alpha);
        b0 = 0.50 * (1.0 - cosW0) * scale;
        b1 = (1.0 - cosW0) * scale;
        b2 = b0;
        a1 = -2.0 * cosW0 * scale;
        a2 = (1.0 - alpha) * scale;
        //Serial.println(b0, 12); Serial.println(b1, 12); Serial.println(b2, 12);
        //Serial.println(a1, 12); Serial.println(a2, 12);
    }
    
    // Set RN seed. Stop audio interrupts if multiple generators are involved
    void setSeed(uint32_t _idum)  {
		idum = _idum;
    }

    virtual void update(void);

private:
    uint32_t idum;
    float sd;

    // Starting IIR coeffs
    // IIR filter turned off:
    double b0 = 1.0;
    double b1 = 0.0;
    double b2 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    // fc = fs/8 and q = 0.7071 (Butterworth 2-pole)
    // double b0 =  0.09763076084032914;
    // double b1 =  0.19526152168065827;
    // double b2 =  0.09763076084032914;
    // double a1 = -0.9428060277021066;
    // double a2 =  0.3333290710634233;

    // Storage for IIR filter
    double yy = 0.0;
    double x0 = 0.0;
    double w1 = 0.0;
    double w2 = 0.0;
};
#endif
