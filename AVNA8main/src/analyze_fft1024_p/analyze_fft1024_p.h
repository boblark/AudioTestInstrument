/* Audio Library for Teensy 3.X - Modified for AVNA
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 * 
 * _p version is modified to
 *   1- Has power as the output quantity (no sqrt)
 *   2- Has low sielobe BlackmanHarris window as default
 *   3- read(first, last) removed
 *   4- checks on bin range removed
 *   5- output[] array made float to streamline
 * for AVNA Spectrum analyzer.  Bob Larkin August 2020
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
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

#ifndef analyze_fft1024_p_h_
#define analyze_fft1024_p_h_

#include "Arduino.h"
#include "AudioStream.h"
#include "arm_math.h"

// windows.c
extern "C" {
extern const int16_t AudioWindowHanning1024[];
extern const int16_t AudioWindowBartlett1024[];
extern const int16_t AudioWindowBlackman1024[];
extern const int16_t AudioWindowFlattop1024[];
extern const int16_t AudioWindowBlackmanHarris1024[];
extern const int16_t AudioWindowNuttall1024[];
extern const int16_t AudioWindowBlackmanNuttall1024[];
extern const int16_t AudioWindowWelch1024[];
extern const int16_t AudioWindowHamming1024[];
extern const int16_t AudioWindowCosine1024[];
extern const int16_t AudioWindowTukey1024[];
}

class AudioAnalyzeFFT1024_p : public AudioStream
{
public:
	AudioAnalyzeFFT1024_p() : AudioStream(1, inputQueueArray),
	  window(AudioWindowBlackmanHarris1024), state(0), outputflag(false) {
		arm_cfft_radix4_init_q15(&fft_inst, 1024, 0, 1);
	}
	bool available() {
		if (outputflag == true) {
			outputflag = false;
			return true;
		}
		return false;
	}
	float read(unsigned int binNumber) {
		if (binNumber > 511) return 0.0;
		return output[binNumber];
	}

	void windowFunction(const int16_t *w) {
		window = w;
	}
	virtual void update(void);
	float32_t output[512];   // rev _p
private:
	void init(void);
	const int16_t *window;
	audio_block_t *blocklist[8];
	int16_t buffer[2048] __attribute__ ((aligned (4)));
	uint8_t state;
	volatile bool outputflag;
	audio_block_t *inputQueueArray[1];
	arm_cfft_radix4_instance_q15 fft_inst;
};

#endif
