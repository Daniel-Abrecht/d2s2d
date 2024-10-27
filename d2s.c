#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

enum {
  BIT_COUNT = 9,
  SAMPLE_COUNT_MIN = BIT_COUNT*2+1, // We need at least this many samples for our data
  SAMPLE_COUNT = SAMPLE_COUNT_MIN + 1, // We add a few extra samples, this gives some tolerance
};

static void write_wav_header(){
  static const unsigned char header[] =
    "RIFF\x24\0\0\x80WAVE"
    "fmt \x10\0\0\0\1\0\1\0\x44\xAC\0\0\0\xEE\2\0\4\0\x20\0"
    "data\0\0\0\x80";
  fwrite(header, 1, sizeof(header)-1, stdout);
}

void write_sample(double x){
  if(x >  1) x =  1;
  if(x < -1) x = -1;
  uint32_t sample = x * 0x7FFFFFFF;
  putchar(sample);
  putchar(sample >>  8);
  putchar(sample >> 16);
  putchar(sample >> 24);
}

static double g_amplitude;

void print_byte(unsigned ch){
  for(int t=0; t<SAMPLE_COUNT; t++){
    double sample = 0;
    for(int b=0; b<BIT_COUNT; b++){ // bits = frequencies to encode
      if(!(ch & (1<<b)))
        continue;
      sample += sin(2.*M_PI*(BIT_COUNT-b)*t/SAMPLE_COUNT); // Note: Highest byte encoded using lowest frequency.
    }
    sample *= g_amplitude;
    write_sample(sample);
  }
}

#define SYNC_SIGNAL 0x100u

int main(){
  write_wav_header();
  g_amplitude = 1; // Only one sine wave
  // No data, baseline
  print_byte(0);
  print_byte(0);
  // For calibration: timing, phase, amplitude and polarity are determined here
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  print_byte(SYNC_SIGNAL);
  // We have up to 9 sign waves adding up.
  // If there is any clipping, the signal gets worse. Same if it's less loud.
  g_amplitude = 0.16;
  print_byte('>' | SYNC_SIGNAL); // Signify start of data
  for(int ch; (ch=getchar())!=EOF; )
    print_byte(ch | SYNC_SIGNAL);
  print_byte(0);
  print_byte(0);
}
