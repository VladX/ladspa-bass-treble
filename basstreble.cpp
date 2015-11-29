/*
Copyright (c) 2015, Vladislav Samsonov
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ladspa-bass-treble nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ladspa.h>
#include <math.h>

#define ARRSIZE(X) (sizeof(X) / sizeof(X[0]))

static const int kPluginID = 5335;
static const unsigned kChannels = 2;
static const unsigned kLowCutoff = 170;
static const unsigned kHighCutoff = 15000;
static const long double PI_LD = 3.141592653589793238462643383279502884L;

static LADSPA_PortDescriptor ladspa_ports[] = {
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
};

static const char * const ladspa_port_names[] = {
	"In (ch #1)",
	"In (ch #2)",
	"Out (ch #1)",
	"Out (ch #2)",
	"Bass",
	"Treble",
	"Amplifier"
};

static const LADSPA_PortRangeHint ladspa_port_ranges[] = {
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{LADSPA_HINT_DEFAULT_0, 0, 0},
	{LADSPA_HINT_DEFAULT_0, 0, 0},
	{LADSPA_HINT_DEFAULT_1, 0, 0}
};

class IIRFilter {
private:
	double xv[4];
	double yv[4];
protected:
	double ax[4];
	double by[4];
public:
	inline IIRFilter () {
		reset();
	}
	
	inline void reset () {
		for (int i = 0; i < ARRSIZE(xv); ++i)
			xv[i] = 0;
		for (int i = 0; i < ARRSIZE(yv); ++i)
			yv[i] = 0;
	}
	
	inline double filter_apply (double sample) {
		xv[0] = xv[1];
		xv[1] = xv[2];
		xv[2] = xv[3];
		xv[3] = sample;
		yv[0] = yv[1];
		yv[1] = yv[2];
		yv[2] = yv[3];
		yv[3] = ax[0] * xv[0] + ax[1] * xv[1] + ax[2] * xv[2] + ax[3] * xv[3] +
				by[0] * yv[0] + by[1] * yv[1] + by[2] * yv[2];
		
		return yv[3];
	}
};

class LPFilter : public IIRFilter {
public:
	inline LPFilter (const long double cutoff, const long double samplerate) {
		const long double ita = 1.0 / tanl(PI_LD * cutoff / samplerate);
		const long double Ainv = 1.0 / (1 + 2 * ita + 2 * ita * ita + ita * ita * ita);
		ax[0] = Ainv;
		ax[1] = 3 * Ainv;
		ax[2] = 3 * Ainv;
		ax[3] = Ainv;
		by[0] = -(1 - 2 * ita + 2 * ita * ita - ita * ita * ita) * Ainv;
		by[1] = -(3 - 2 * ita - 2 * ita * ita + 3 * ita * ita * ita) * Ainv;
		by[2] = -(3 + 2 * ita - 2 * ita * ita - 3 * ita * ita * ita) * Ainv;
	}
};

class HPFilter : public IIRFilter {
public:
	inline HPFilter (const long double cutoff, const long double samplerate) {
		const long double ita = tanl(PI_LD * cutoff / samplerate);
		const long double Ainv = 1.0 / (1 + 2 * ita + 2 * ita * ita + ita * ita * ita);
		ax[0] = Ainv;
		ax[1] = -3 * Ainv;
		ax[2] = 3 * Ainv;
		ax[3] = -Ainv;
		by[0] = (1 - 2 * ita + 2 * ita * ita - ita * ita * ita) * Ainv;
		by[1] = -(3 - 2 * ita - 2 * ita * ita + 3 * ita * ita * ita) * Ainv;
		by[2] = (3 + 2 * ita - 2 * ita * ita - 3 * ita * ita * ita) * Ainv;
	}
};

class EqInstance {
	LPFilter lp[kChannels];
	HPFilter hp[kChannels];
public:
	const LADSPA_Data * in[kChannels];
	LADSPA_Data * out[kChannels];
	LADSPA_Data * bass;
	LADSPA_Data * treble;
	LADSPA_Data * amp;
	
	EqInstance (unsigned long sr) : lp{LPFilter(kLowCutoff, sr), LPFilter(kLowCutoff, sr)}, hp{HPFilter(kHighCutoff, sr), HPFilter(kHighCutoff, sr)} {}
	
	inline void reset () {
		for (unsigned i = 0; i < kChannels; ++i)
			lp[i].reset();
		for (unsigned i = 0; i < kChannels; ++i)
			hp[i].reset();
	}
	
	inline void run (unsigned long count) {
		const double kBass = * bass;
		const double kTreble = * treble;
		const double kAmp = * amp;
		for (unsigned ch = 0; ch < kChannels; ++ch) {
			for (unsigned long i = 0; i < count; ++i) {
				const double low = lp[ch].filter_apply(in[ch][i]);
				const double high = hp[ch].filter_apply(in[ch][i]);
				out[ch][i] = in[ch][i] + low * kBass + high * kTreble;
				out[ch][i] *= kAmp;
			}
		}
	}
};

static LADSPA_Handle instantiate (const LADSPA_Descriptor *, unsigned long sample_rate) {
	return new EqInstance(sample_rate);
}

static void connect_port (LADSPA_Handle ptr, unsigned long port, LADSPA_Data * d) {
	EqInstance * instance = (EqInstance *) ptr;
	switch (port) {
		case 0:
		case 1:
			instance->in[port] = d;
			break;
		case 2:
			instance->out[0] = d;
			break;
		case 3:
			instance->out[1] = d;
			break;
		case 4:
			instance->bass = d;
			break;
		case 5:
			instance->treble = d;
			break;
		case 6:
			instance->amp = d;
			break;
	}
}

static void activate (LADSPA_Handle ptr) {
	EqInstance * instance = (EqInstance *) ptr;
	instance->reset();
}

static void run (LADSPA_Handle ptr, unsigned long count) {
	EqInstance * instance = (EqInstance *) ptr;
	instance->run(count);
}

static void cleanup (LADSPA_Handle ptr) {
	EqInstance * instance = (EqInstance *) ptr;
	delete instance;
}

static LADSPA_Descriptor dsc = {
	kPluginID,
	"bass-treble",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Simple 2-band equalizer (bass/treble)",
	"Vladislav Samsonov",
	"Copyright (c) 2015, Vladislav Samsonov <vvladxx@gmail.com>",
	ARRSIZE(ladspa_ports),
	ladspa_ports,
	ladspa_port_names,
	ladspa_port_ranges,
	0,
	instantiate,
	connect_port,
	activate,
	run,
	0,
	0,
	0,
	cleanup
};

extern "C" const LADSPA_Descriptor * ladspa_descriptor (unsigned long i) {
	if (i == 0)
		return &dsc;
	return 0;
}
