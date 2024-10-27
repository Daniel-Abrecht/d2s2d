#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <tgmath.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

enum {
  BIT_COUNT = 9,
  SAMPLE_COUNT_MIN = BIT_COUNT*2+1, // We need at least this many samples for our data
};

//////////////////////////////////////////////////////////////
// A fourier transform, Based on discrete fourier transform //
//////////////////////////////////////////////////////////////

struct fourier {
  short i; // Current sample index for compareason frequencies.
  const short frequency_count;
  // Must be at least frequency_count*2+1
  // Usually, people use an FFT and infer frequency_count from sample_count or vice versa,
  // but we want to handle cases where we've got more samples than we need.
  short sample_count;
  float sincos_components[0]; // Nonstandard, but necessary for how we use this, for alignment & padding reasons
};

#define DECODER_STATE \
  X(DECODER_INIT) \
  X(DECODER_DETECT_POLARITY) \
  X(DECODER_DETECT_WAVE_FIRST_HALF) \
  X(DECODER_DETECT_WAVE_SECOND_HALF) \
  X(DECODER_DETECT_CALIBRATE) \
  X(DECODER_DECODE_DATA) \
  X(DECODER_EOF)

enum decoder_state {
#define X(Y) Y,
  DECODER_STATE
#undef X
};
const char*const decoder_state_str[] = {
#define X(Y) [Y] = #Y,
  DECODER_STATE
#undef X
};
#undef DECODER_STATE

struct decoder {
  enum decoder_state state;
  // Polarity and level of signal
  bool polarity;
  int16_t phase;
  int16_t phase2;
  int16_t phase3;
  uint16_t baseline;
  uint16_t signal_max;
  uint16_t signal_min;
  struct { // Fourier state
    struct fourier fourier;
    // sine / cosine components of frequency signal. Only 1..7, excluding frequency 0 (amplitude), excluding frequencies 10..17
    float sincos_components[BIT_COUNT][2];
  };
};
static_assert(offsetof(struct decoder, fourier)+sizeof(struct fourier) == offsetof(struct decoder, sincos_components), "Member sincos_components not directly following struct fourier");

static inline float nsin(float f){
  return sin(f * 2 * M_PI);
}

static inline float ncos(float f){
  return nsin(f+0.25);
}

static inline float sincos_to_phase(float x, float y){
  return atan2(y,x) / (2*M_PI);
}

static inline float quad(float x){
  return x*x;
}

bool fourier_add_sample(struct fourier*const fourier, const float sample){
  float(*const sincos_components)[2] = (float(*)[2])(fourier+1);
  for(int f=0; f<fourier->frequency_count; f++){
    float i = (float)((f+1)*fourier->i) / fourier->sample_count;
    sincos_components[f][0] += nsin(i) * sample * 25 / fourier->sample_count;
    sincos_components[f][1] += ncos(i) * sample * 25 / fourier->sample_count;
  }
  return ++fourier->i >= fourier->sample_count;
}

// Note: returned frequency is still squared here
void fourier_to_frequency(struct fourier*const fourier, float fph[]){
  float(*const sincos_components)[2] = (float(*)[2])(fourier+1);
  for(int f=0; f<fourier->frequency_count; f++)
    fph[f] = quad(sincos_components[f][0]) + quad(sincos_components[f][1]);
}

void fourier_reset(struct fourier*const fourier){
  float(*const sincos_components)[2] = (float(*)[2])(fourier+1);
  memset(sincos_components, 0, sizeof(*sincos_components)*fourier->frequency_count);
  fourier->i = 0;
}

enum {
  DECODER_RET_EOF = -1,
  DECODER_RET_NO_DATA = -2,
  DECODER_RET_ERROR = -3,
};

int decoder_decode_byte(struct decoder* decoder, float sample){
  if(fourier_add_sample(&decoder->fourier, sample)){
    float frequency[decoder->fourier.frequency_count];
    fourier_to_frequency(&decoder->fourier, frequency);
    // TODO: adjust sample count and skip some if necessary based on phase, and total chunk time.
    unsigned byte = 0;
    for(int f=0; f<decoder->fourier.frequency_count; f++){
      if(frequency[f] > 0.5*0.5)
        byte |= 1u<<(BIT_COUNT-f-1);
      // fprintf(stderr,"%.2f ", /*sqrt*/(frequency[f]));
    }
    // fprintf(stderr,"\n");
    // fprintf(stderr,"f %X\n", byte);
    if(byte & 0x100u){
      // We use the lowest frequency for adjustments. One full wavelength includes all the samples.
      const float phase = sincos_to_phase(decoder->sincos_components[0][0], decoder->sincos_components[0][1]);
      decoder->phase = round(phase * decoder->fourier.sample_count);
      // fprintf(stderr,"> %f %d\n", phase, decoder->phase);
    }else{
      decoder->phase = 0;
    }
    fourier_reset(&decoder->fourier);
    if(byte == 0)
      return DECODER_RET_EOF;
    return byte & 0xFF;
  }
  return DECODER_RET_NO_DATA;
}

enum { TIMING_SIGNAL_THRESHOLD = 64 };

static inline void decoder_update_magnitude(struct decoder*const decoder, const uint16_t sample){
  if(decoder->signal_max < sample)
    decoder->signal_max = sample;
  if(decoder->signal_min > sample)
    decoder->signal_min = sample;
}

int decoder_decode(struct decoder*const decoder, const uint16_t sample){
  // if(decoder->state != DECODER_EOF)
  //   fprintf(stderr,"%s: %c %u < %u < %u: %u\n", decoder_state_str[decoder->state], decoder->polarity?'+':'-', decoder->signal_min, decoder->baseline, decoder->signal_max, sample);
  float fsample;
  if(decoder->state >= DECODER_DETECT_CALIBRATE){
    fsample = (float)(sample - decoder->signal_min) / (decoder->signal_max - decoder->signal_min);
    if(!decoder->polarity)
      fsample = 1.f-fsample;
  }
  switch(decoder->state){
    case DECODER_INIT: {
      decoder->baseline = sample;
      decoder->state = DECODER_DETECT_POLARITY;
      decoder->fourier.sample_count = 0;
    } break;
    case DECODER_DETECT_POLARITY: {
      int diff = sample - decoder->baseline;
      decoder_update_magnitude(decoder, sample);
      if(diff > TIMING_SIGNAL_THRESHOLD || diff < -TIMING_SIGNAL_THRESHOLD){
        decoder->polarity = diff > 0;
        decoder->state = DECODER_DETECT_WAVE_FIRST_HALF;
        decoder->signal_max = decoder->baseline;
        decoder->signal_min = decoder->baseline;
      }else{
        decoder->baseline = (int)decoder->baseline + diff / 8;
        break;
      }
    } /* fallthrough */
    case DECODER_DETECT_WAVE_FIRST_HALF: {
      decoder->fourier.sample_count++;
      int diff = decoder->polarity ? decoder->signal_max - sample : sample - decoder->signal_min;
      if(diff > decoder->signal_max - decoder->signal_min)
        decoder->state = DECODER_DETECT_WAVE_SECOND_HALF;
      decoder_update_magnitude(decoder, sample);
    } break;
    case DECODER_DETECT_WAVE_SECOND_HALF: {
      decoder->fourier.sample_count++;
      decoder_update_magnitude(decoder, sample);
      //fprintf(stderr,"! %c %u\n", decoder->polarity?'+':'-', (decoder->signal_max + decoder->signal_min) / 2);
      if((sample > (decoder->signal_max + decoder->signal_min) / 2) == decoder->polarity){
        // Note: sample_count is a very, very rough estimate
        if(decoder->fourier.sample_count < SAMPLE_COUNT_MIN)
          decoder->fourier.sample_count = SAMPLE_COUNT_MIN;
        decoder->state = DECODER_DETECT_CALIBRATE;
        decoder->phase = 0;
        decoder->phase2 = 0;
        decoder->phase3 = 0;
      }
    } break;
    case DECODER_DETECT_CALIBRATE: {
      // fprintf(stderr, "> %d\n", decoder->fourier.sample_count);
      // decoder->state = DECODER_EOF;
      if(decoder->phase < 0){
        decoder->phase++;
        break;
      }
      int byte = decoder_decode_byte(decoder, fsample);
      if(byte == DECODER_RET_EOF){
        // False positive, retry.
        decoder->state = DECODER_INIT;
        break;
      }
      if(byte >= 0){
        // fprintf(stderr, "!! %d\n", decoder->phase);
        if(decoder->phase && decoder->phase2 && decoder->phase3 && (decoder->phase < 0) == (decoder->phase2 < 0) && (decoder->phase2 < 0) == (decoder->phase3 < 0)){
          decoder->fourier.sample_count -= (decoder->phase + decoder->phase2 + decoder->phase3) / 3;
          decoder->phase2 = 0;
        }else{
          decoder->phase3 = decoder->phase2;
          decoder->phase2 = decoder->phase;
        }
        if(byte == '>'){ // start byte
          decoder->state = DECODER_DECODE_DATA;
        }
        if(decoder->phase > 0)
          decoder_decode_byte(decoder, fsample);
      }
    } break;
    case DECODER_DECODE_DATA: {
      if(decoder->phase < 0){
        decoder->phase++;
        break;
      }
      int byte = decoder_decode_byte(decoder, fsample);
      if(byte == DECODER_RET_EOF)
        decoder->state = DECODER_EOF;
      if(byte >= 0){
        if(decoder->phase && decoder->phase2 && decoder->phase3 && (decoder->phase < 0) == (decoder->phase2 < 0) && (decoder->phase2 < 0) == (decoder->phase3 < 0)){
          decoder->fourier.sample_count -= (decoder->phase + decoder->phase2 + decoder->phase3) / 3;
          decoder->phase2 = 0;
        }else{
          decoder->phase3 = decoder->phase2;
          decoder->phase2 = decoder->phase;
        }
        if(decoder->phase > 0)
          decoder_decode_byte(decoder, fsample);
      }
      return byte;
    } break;
    case DECODER_EOF: return DECODER_RET_EOF;
  }
  return DECODER_RET_NO_DATA;
}

#define SIGNAL_STREANGTH 1024

int main(){
  struct decoder decoder = {
    .fourier.frequency_count = BIT_COUNT,
  };
  // TODO: determine speed
  for(int32_t x; fread(&x, 4, 1, stdin)>0; ){
    float sample = (float)x / 0x80000000lu;
    int byte = decoder_decode(&decoder, (sample+1)/2*SIGNAL_STREANGTH);
    if(byte >= 0){
      // fprintf(stderr,"%02X\n", byte);
      putchar(byte);
      // TODO: handle next sync signal
    }
    if(byte == DECODER_RET_EOF)
      break;
  }
}
