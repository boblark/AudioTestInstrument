/*
 *  synth_GaussianWhiteNoiseR2.cpp
 * This is an R2 frozen copy of synth_GaussianWhiteNoise for the AVNA
 *  by Bob Larkin  W7PUA 4 Oct 2020
 *  CREDITS - See companion synth_GaussianWhiteNoiseR2.h
 *
 * Copyright (c) 2020 Robert Larkin
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
// See the following for details
#include "synth_GaussianWhiteNoiseR2.h"

void AudioSynthNoiseGaussian::update(void) {  
#define ABS(X)   ((X>=0)? X : -(X) )
#define ROUND(X) (X>=0)? (int) (X + 0.5) : (int)-(ABS(X) +0.5)

    audio_block_t *blockOut;
    uint32_t it;
    float rdev = 0.0f;
    float gwn_f32, y_f32;
    int16_t* pd;

    blockOut = allocate();
    if (!blockOut) return;
    pd = &blockOut->data[0];  // Ptr to write data (128 16-bit ints)

    if (sd < 0.00001f) {
        for(int i=0; i<AUDIO_BLOCK_SAMPLES; i++)
            *pd++ = 0;
        transmit(blockOut);
        release(blockOut);
        return;  // Not enabled, spend minimal time here
    }

    union {               // Trick for creating float
        uint32_t  i32;
        float f32;
        } uinf;

    for(int i=0; i<AUDIO_BLOCK_SAMPLES; i++)  {
        rdev = 0.0f;
        for (int j=0; j<12; j++){   // Add 12, using Central Limit to get Gaussian
            idum = (uint32_t)1664525 * idum + (uint32_t)1013904223;
            it = FL_ONE | (FL_MASK & idum);  // Generate uniform random number
            // rdev += (*(float *)&it) - 1.0f;  // Cute Knuth/Lewis convert to float,
            // but it gets compile warning, so instead we use union as
            uinf.i32 = it;      // union puts float result to uinf.f32
            rdev += uinf.f32 - 1.0f;  // Accumulate 12 uniform rd's
        }
        // A uniform distribution from 0 to 1 has a statistical variance of 1/12. If
        // we add 12 of these together, we will have a variance of 1.0 and thus a standard
        // deviation (sd) of sqrt(1.0) = 1.0.  So to make an sd of, say 0.1, and
        // a mean of 0.0, we subtract (12 * 0.5) and then multiply by sd=0.1:
        gwn_f32 = sd*(rdev - 6.0f);

        // 2-pole LP filter for those that want to limit the bandwidth
        if (b0 > 0.95)   // Set frequency near fs/2 and we do no filtering at all
          y_f32 = gwn_f32;
        else
          {
          // https://www.advsolned.com/implementing-biquad-iir-filters-with-the-asn-filter-designer-and-the-arm-cmsis-dsp-software-framework/
          // Use Direct Form II of BiQuad IIR
          // w1 and w2 are internal values from previous passes
          x0 = (double) gwn_f32;
          yy = b0*x0 + w1;   // y=IIR filter output, current w1
          w1 = b1*x0 - a1*yy + w2;        // update to next w1
          w2 = b2*x0 - a2*yy;             // update to next w2
          y_f32 = (float)yy;
	      }
        // Convert to 16-bit signed integer
        if (y_f32>0.9999695f)
            *pd++ = 32767;
        else if (y_f32<-1.0f)
            *pd++ = -32768;
        else
            *pd++ = ROUND(32768.0f*y_f32);  // Convert to int16_t
    }
    transmit(blockOut);
    release(blockOut); 
}
