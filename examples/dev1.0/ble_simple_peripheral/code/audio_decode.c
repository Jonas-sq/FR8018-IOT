#include "audio_decode.h"
#include "adpcm_ms.h"
#include "adpcm_ima.h"
#include "lwrb.h"
#include "string.h"
#include "co_printf.h"

#include "driver_i2s.h"
#include "driver_codec.h"
#include "driver_pmu.h"
#include "driver_gpio.h"
#include "os_mem.h"
#include "driver_flash.h"

#define AUDIO_DATA_LEN  		(1024 * 6)
#define ENCODE_DATA_BLOCK_SIZE  512
#define DECODE_DATA_BLOCK_SIZE  ((ENCODE_DATA_BLOCK_SIZE + 8) * 4)

uint8_t audio_encode_data_block[ENCODE_DATA_BLOCK_SIZE];
uint16_t audio_decode_data_bolck[DECODE_DATA_BLOCK_SIZE];
ADPCMContext ms_adpcm_context;


uint16_t lwrb_audio_data[AUDIO_DATA_LEN];
lwrb_t lwrb_audio_header;
Wav_fotmat_info_t user_wav_fotmat_info,user_wav_fotmat_info_backup;

uint8_t audio_mode = 0;

void user_lwrb_init(void)
{
	lwrb_init(&lwrb_audio_header, lwrb_audio_data, sizeof(lwrb_audio_data));
	lwrb_reset(&lwrb_audio_header);
}


static void audio_flash_read(uint32_t falsh_addr, uint32_t size, uint8_t *buff)
{
	flash_read(falsh_addr, size, buff);  //flash函数，用户实现
//	W25QXX_Read(buff,falsh_addr,size);
}

static uint8_t RIFF_header(uint8_t *p_header_org,Wav_fotmat_info_t *p_wav)
{
    uint8_t *header = p_header_org;
    if(memcmp(header, "RIFF", 4) == 0)
    {
		memcpy((uint8_t *)&p_wav->wav_format.riff, (uint8_t *)header, sizeof(p_wav->wav_format.riff));
		header += sizeof(p_wav->wav_format.riff);
		
        if(memcmp(header, "fmt ", 4) == 0)
        {
            memcpy((uint8_t *)&p_wav->wav_format.fmt, (uint8_t *)header, sizeof(p_wav->wav_format.fmt));	
			header += 8;
			header += p_wav->wav_format.fmt.Subchunk1Size;
			
			while(memcmp(header-4, "data", 4))
			{
				header++;
			}
			
			memcpy((uint8_t *)&p_wav->wav_format.data, (uint8_t *)(header-4), sizeof(p_wav->wav_format.data));
			p_wav->data_start_offset = header + 4 - p_header_org;
			p_wav->data_stop_offset = p_wav->data_start_offset + p_wav->wav_format.data.Subchunk2Size;
			return 0;
        }
		else
		{
		    co_printf("fmt  error\r\n");
			return 1;
		}
    }
    else
    {
		co_printf("RIFF error\r\n");
        return 1;
    }
}

//获取音频头文件信息
uint8_t audio_wave_header(uint32_t flash_addr)
{
   uint8_t buff[0xff];
   uint8_t result = 0;
	
    audio_flash_read(flash_addr, 0xff, buff);
  
    memset(&user_wav_fotmat_info, 0, sizeof(user_wav_fotmat_info));
	user_wav_fotmat_info.audio_data_flash_addr = flash_addr;
    result = RIFF_header(buff, &user_wav_fotmat_info);
	memcpy(&user_wav_fotmat_info_backup, &user_wav_fotmat_info, sizeof(user_wav_fotmat_info));
	if(result)
	{
		co_printf("audio no data\r\n");
		return 1;
	}
	
	ms_adpcm_context.channel = user_wav_fotmat_info.wav_format.fmt.NumChannels;
	ms_adpcm_context.block_align = user_wav_fotmat_info.wav_format.fmt.BlockAlign;
	
	return 0;
}


//有效音频数据解码
uint8_t audio_data_decode(void)
{
//	uint8_t len = 0;
//	static uint32_t decode_data_len;
//	uint8_t *p_encode_data = NULL;
//	uint8_t *p_decode_data = NULL;
//	p_encode_data = (uint8_t *)os_zalloc(user_wav_fotmat_info.wav_format.fmt.BlockAlign);
//	
//	if(user_wav_fotmat_info.data_start_offset >= user_wav_fotmat_info.data_stop_offset) 
//	{
//		return 1;
//	}		

//	if((user_wav_fotmat_info.data_start_offset + user_wav_fotmat_info.wav_format.fmt.BlockAlign) <= user_wav_fotmat_info.data_stop_offset) 
//	{
//	   audio_flash_read(user_wav_fotmat_info.audio_data_flash_addr + user_wav_fotmat_info.data_start_offset, user_wav_fotmat_info.wav_format.fmt.BlockAlign, p_encode_data); 
//	   user_wav_fotmat_info.data_start_offset += user_wav_fotmat_info.wav_format.fmt.BlockAlign;
//	}
//	else 
//	{
//	   len = user_wav_fotmat_info.data_stop_offset - user_wav_fotmat_info.data_start_offset;
//	   audio_flash_read(user_wav_fotmat_info.audio_data_flash_addr + user_wav_fotmat_info.data_start_offset, len, p_encode_data); 
//	   user_wav_fotmat_info.data_start_offset += len;	
//	}
//	p_decode_data = (uint8_t *)os_zalloc((user_wav_fotmat_info.wav_format.fmt.BlockAlign + 8) * 4);
//	
//	decode_data_len = (user_wav_fotmat_info.wav_format.fmt.BlockAlign + 8) * 4;
//	adpcm_decode_frame(&ms_adpcm_context,
//                       (short *)p_decode_data, (int *)&decode_data_len,
//                       p_encode_data, user_wav_fotmat_info.wav_format.fmt.BlockAlign);

//	
//	lwrb_write(&lwrb_audio_header, p_decode_data, decode_data_len);
//	os_free(p_encode_data);
//	os_free(p_decode_data);				   
//	return 0;


	uint16_t len = 0;
    static uint32_t decode_data_len;
	
	memset(&audio_encode_data_block, 0, sizeof(audio_encode_data_block));
	if(user_wav_fotmat_info.data_start_offset >= user_wav_fotmat_info.data_stop_offset) 
	{
		if(audio_mode == 1) //单曲循环播放
		{
			memcpy(&user_wav_fotmat_info, &user_wav_fotmat_info_backup, sizeof(user_wav_fotmat_info));
		}
		else
		{
			return 1;
		}
	}		

	if(user_wav_fotmat_info.wav_format.fmt.AudioFormat == 0x01)
	{
		user_wav_fotmat_info.wav_format.fmt.BlockAlign = 256;
	}
	if((user_wav_fotmat_info.data_start_offset + user_wav_fotmat_info.wav_format.fmt.BlockAlign) <= user_wav_fotmat_info.data_stop_offset) 
	{
		
		len = user_wav_fotmat_info.wav_format.fmt.BlockAlign;
	   audio_flash_read(user_wav_fotmat_info.audio_data_flash_addr + user_wav_fotmat_info.data_start_offset, len, audio_encode_data_block); 
	   user_wav_fotmat_info.data_start_offset += len;
		
	}
	else 
	{
	   len = user_wav_fotmat_info.data_stop_offset - user_wav_fotmat_info.data_start_offset;
	   audio_flash_read(user_wav_fotmat_info.audio_data_flash_addr + user_wav_fotmat_info.data_start_offset, len, audio_encode_data_block); 
	   user_wav_fotmat_info.data_start_offset += len;	
	}

	decode_data_len = ((user_wav_fotmat_info.wav_format.fmt.BlockAlign + 8) * 4);
	
	if(user_wav_fotmat_info.wav_format.fmt.AudioFormat == 0x01)
	{
		memcpy(audio_decode_data_bolck, audio_encode_data_block, len);//pcm
		lwrb_write(&lwrb_audio_header, audio_decode_data_bolck, len);  
	}
	else if(user_wav_fotmat_info.wav_format.fmt.AudioFormat == 0x02)
	{
		memset(&audio_decode_data_bolck,0,sizeof(audio_decode_data_bolck));
		adpcm_decode_frame(&ms_adpcm_context,
					   (short *)audio_decode_data_bolck, (int *)&decode_data_len,
					   audio_encode_data_block, len);  //ms-adpcm
	    lwrb_write(&lwrb_audio_header, audio_decode_data_bolck, decode_data_len);  
	}
	else if(user_wav_fotmat_info.wav_format.fmt.AudioFormat == 0x11)
	{
		memset(&audio_decode_data_bolck,0,sizeof(audio_decode_data_bolck));
		adpcm_decode_block ((short *)audio_decode_data_bolck, audio_encode_data_block, len, 1);  //iam-adpcm
		lwrb_write(&lwrb_audio_header, audio_decode_data_bolck, decode_data_len);  
	}
	return 0;
}

uint16_t get_audio_simplerate(void)
{
	return user_wav_fotmat_info.wav_format.fmt.SampleRate;
}
