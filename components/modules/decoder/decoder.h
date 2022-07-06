#ifndef  __DECODE_H
#define  __DECODE_H

#include "os_timer.h"
#include "co_printf.h"
#include <stdint.h>
#include "co_list.h"

#include "lwrb.h"
void user_lwrb_init(void);
typedef struct
{
    uint32_t start_base;
    uint32_t tot_data_len;
    uint32_t last_offset;
    uint8_t last_page_idx;
} sbc_store_info_t;

typedef struct
{
    uint32_t last_read_offset;
    uint8_t last_read_page_idx;
    uint8_t end_flag;

    uint32_t store_data_len;
    uint16_t sbc_frame_len;
    uint16_t sbc_data_tot_len;
} speaker_env_t;

extern speaker_env_t speaker_env ;

extern sbc_store_info_t sbc_sotre_env;

#define FLASH_PAGE_SIZE (0x1000)
#define USER_FLASH_BASE_ADDR (0x60000)


struct decoder_prepare_t
{
    uint32_t data_start;
    uint32_t data_end;
    uint32_t tot_data_len;
    uint32_t start_offset;
    uint8_t store_type;

    uint16_t frame_len;
};

struct decoder_pcm_t
{
    struct co_list_hdr list;
    uint16_t pcm_size;
    uint16_t pcm_offset;


    uint16_t pcm_data[1];

};

struct decoder_env_t
{
    struct co_list pcm_buffer_list;

    void * decoder_context;
    uint32_t data_start;
    uint32_t data_end;
    uint32_t tot_data_len;
    uint32_t data_processed_len;

    uint32_t current_pos;
    uint16_t frame_len;
    uint8_t store_type;
    uint8_t pcm_buffer_counter;
};
extern struct decoder_env_t decoder_env;
extern uint8_t stop_flag ;
enum decoder_state_t
{
    DECODER_STATE_IDLE,
    DECODER_STATE_BUFFERING,
    DECODER_STATE_PLAYING,
    DECODER_STATE_WAITING_END,

    DECODER_STATE_MAX,
};

extern uint8_t decodeTASKState;
extern bool decoder_hold_flag;
extern uint8_t *sbc_buff ;
extern uint8_t Flash_data_state;



uint8_t audio_wave_header(uint32_t flash_addr);

void decoder_play_next_frame_handler(void *arg);
void decoder_play_next_frame(void);
//void test_speaker_from_flash(void);
void decoder_end_func(void);
void test_end_speaker(void);

uint16_t get_audio_simplerate(void);
	
typedef struct WAV_data {
    /* sub-chunk "data" */
    char Subchunk2ID[4];   /* "data" */
    /* sub-chunk-size */
    uint32_t Subchunk2Size; /* data size */
    /* sub-chunk-data */
//    Data_block_t block;
} Data_t;

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

//有效音频数据解码
uint8_t audio_data_decode(void);


typedef struct WAV_RIFF {
    /* chunk "riff" */
    char ChunkID[4];   /* "RIFF" */
    /* sub-chunk-size */
    uint32_t ChunkSize; /* 36 + Subchunk2Size */
    /* sub-chunk-data */
    char Format[4];    /* "WAVE" */
} RIFF_t;

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

enum 
{
  AUDIO_PLAY_START,
  AUDIO_PLAY_NEXT_FRAME,
  AUDIO_PLAY_STOP,	
};
extern lwrb_t lwrb_audio_header;

#endif

