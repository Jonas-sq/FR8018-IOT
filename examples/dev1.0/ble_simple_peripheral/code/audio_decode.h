#ifndef _AUDIO_ENCODE_H_

#define _AUDIO_ENCODE_H_

#include "lwrb.h"


enum 
{
  AUDIO_PLAY_START,
  AUDIO_PLAY_NEXT_FRAME,
  AUDIO_PLAY_STOP,	
};

typedef struct WAV_RIFF {
    /* chunk "riff" */
    char ChunkID[4];   /* "RIFF" */
    /* sub-chunk-size */
    uint32_t ChunkSize; /* 36 + Subchunk2Size */
    /* sub-chunk-data */
    char Format[4];    /* "WAVE" */
} RIFF_t;

typedef struct WAV_FMT {
    /* sub-chunk "fmt" */
    char Subchunk1ID[4];   /* "fmt " */
    /* sub-chunk-size */
    uint32_t Subchunk1Size; /* 16 for PCM */
    /* sub-chunk-data */
    uint16_t AudioFormat;   /* PCM = 1*/
    uint16_t NumChannels;   /* Mono = 1, Stereo = 2, etc. */
    uint32_t SampleRate;    /* 8000, 44100, etc. */
    uint32_t ByteRate;  /* = SampleRate * NumChannels * BitsPerSample/8 */
    uint16_t BlockAlign;    /* = NumChannels * BitsPerSample/8 */
    uint16_t BitsPerSample; /* 8bits, 16bits, etc. */
} FMT_t;

typedef struct WAV_data {
    /* sub-chunk "data" */
    char Subchunk2ID[4];   /* "data" */
    /* sub-chunk-size */
    uint32_t Subchunk2Size; /* data size */
    /* sub-chunk-data */
//    Data_block_t block;
} Data_t;


typedef struct WAV_fotmat {
   RIFF_t riff;
   FMT_t fmt;
   Data_t data;
} Wav_t;

typedef struct WAV_fotmat_info {
   Wav_t  wav_format;     
   uint32_t audio_data_flash_addr; //存放音频的flash地址
   uint32_t data_start_offset;  //有效音频起始偏移地址(相对audio_data_flash_addr)
   uint32_t data_stop_offset;  //有效音频结束偏移地址 (相对audio_data_flash_addr)
} Wav_fotmat_info_t;

extern lwrb_t lwrb_audio_header;

void user_lwrb_init(void);
uint8_t audio_wave_header(uint32_t flash_addr);
void audio_get_flash_addr(uint32_t flash_addr);
uint8_t audio_data_decode(void);
uint16_t get_audio_simplerate(void);
#endif

