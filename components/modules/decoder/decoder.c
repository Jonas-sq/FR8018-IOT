/**
 * Copyright (c) 2019, Freqchip
 * 
 * All rights reserved.
 * 
 * 
 */

/*
 * INCLUDES
 */
#include "decoder.h"
#include "driver_plf.h"
#include "core_cm3.h"
#include "os_task.h"
#include "os_msg_q.h"
#include "speaker.h"
#include "driver_flash.h"
#include "co_printf.h"
#include <string.h>
#include "os_mem.h"
#include "adpcm_ms.h"
#include "adpcm_ima.h"
#include "user_task.h"

#define AUDIO_DATA_LEN  		(1024 * 6)
#define ENCODE_DATA_BLOCK_SIZE  512
#define DECODE_DATA_BLOCK_SIZE  ((ENCODE_DATA_BLOCK_SIZE + 8) * 4)
lwrb_t lwrb_audio_header;
uint8_t audio_encode_data_block[ENCODE_DATA_BLOCK_SIZE];
uint16_t audio_decode_data_bolck[DECODE_DATA_BLOCK_SIZE];
Wav_fotmat_info_t user_wav_fotmat_info,user_wav_fotmat_info_backup;
uint8_t audio_mode = 0;
ADPCMContext ms_adpcm_context;
/*
 * MACROS
 */
#define DEC_DBG FR_DBG_OFF
#define DEC_LOG FR_LOG(DEC_DBG)

/*
 * CONSTANTS 
 */
#define DECODER_STORE_TYPE_RAM      0
#define DECODER_STORE_TYPE_FLASH    1
 
/*
 * TYPEDEFS 
 */

/*
 * GLOBAL VARIABLES 
 */
sbc_store_info_t sbc_sotre_env = {0};
uint8_t *sbc_buff = NULL;
speaker_env_t speaker_env = {0};
bool decoder_hold_flag = false;
struct decoder_env_t decoder_env;
uint8_t stop_flag = 0;
uint8_t decodeTASKState = DECODER_STATE_IDLE;//函数decoder_play_next_frame_handler 中的状态
uint8_t Flash_data_state = true;
uint16_t lwrb_audio_data[AUDIO_DATA_LEN];
/*
 * LOCAL VARIABLES 
 */


/*
 * LOCAL FUNCTIONS
 */

/*
 * EXTERN FUNCTIONS
 */

/*
 * PUBLIC FUNCTIONS
 */
 //////////////ok
static void audio_flash_read(uint32_t falsh_addr, uint32_t size, uint8_t *buff)
{
	flash_read(falsh_addr, size, buff);  //flash函数，用户实现
//	W25QXX_Read(buff,falsh_addr,size);
}

//////////////ok
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

//获取音频头文件信息ok
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

void user_lwrb_init(void)
{
	lwrb_init(&lwrb_audio_header, lwrb_audio_data, sizeof(lwrb_audio_data));
	lwrb_reset(&lwrb_audio_header);
}

/*********************************************************************
 * @fn		decoder_calc_adpcm_ms_frame_len
 *
 * @brief	Get adpcm frame len
 *
 * @param	header_org     - Pointer to pointer to ADPCM audio data
 *
 * @return	None.
 */

uint16_t decoder_calc_adpcm_ms_frame_len(uint8_t **header_org)
{
    uint32_t len;
    uint8_t *header = *header_org;
    uint16_t frame_size;

    if(memcmp(header, "RIFF", 4) == 0)
    {
        header += 12;
        if(memcmp(header, "fmt ", 4) == 0)
        {
            header += 4;
            len = header[0];
            len |= (header[1] << 8);
            len |= (header[2] << 16);
            len |= (header[3] << 24);

            frame_size = header[16];
            frame_size |= (header[17] << 8);
            header += 4;
            header += len;
            while(memcmp(header, "data", 4) != 0)
            {
                header += 4;
                len = header[0];
                len |= (header[1] << 8);
                len |= (header[2] << 16);
                len |= (header[3] << 24);
                header += (len + 4);
            }
            header += 8;
            DEC_LOG("decoder_calc_adpcm_ms_frame_len: %08x, %08x.\r\n", header, *header_org);
            *header_org = header;
            return frame_size;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

/*********************************************************************
 * @fn		read_sbc_from_flash
 *
 * @brief	Get adpcm frame len
 *
 * @param	header_org     - Pointer to pointer to ADPCM audio data
 *
 * @return	None.
 */
int read_sbc_from_flash(uint8_t * sbc_buff, uint32_t read_len)
{
    uint32_t r_len = read_len;
    uint32_t pos = 0;
    while(r_len > 0)
    {
        if( speaker_env.last_read_page_idx < sbc_sotre_env.last_page_idx )
        {
            if( (r_len + speaker_env.last_read_offset) >= FLASH_PAGE_SIZE )
            {
                flash_read(sbc_sotre_env.start_base + speaker_env.last_read_page_idx * FLASH_PAGE_SIZE + speaker_env.last_read_offset
                           ,FLASH_PAGE_SIZE - speaker_env.last_read_offset, sbc_buff + pos);
                r_len -= (FLASH_PAGE_SIZE - speaker_env.last_read_offset);
                pos += (FLASH_PAGE_SIZE - speaker_env.last_read_offset);
                speaker_env.last_read_offset = 0;
                speaker_env.last_read_page_idx++;
            }
            else
            {
                flash_read(sbc_sotre_env.start_base + speaker_env.last_read_page_idx * FLASH_PAGE_SIZE + speaker_env.last_read_offset
                           ,r_len, sbc_buff + pos);
                pos += r_len;
                speaker_env.last_read_offset += r_len;
                r_len = 0;
            }
        }
        else if( speaker_env.last_read_page_idx == sbc_sotre_env.last_page_idx )
        {
            if( speaker_env.last_read_offset >= sbc_sotre_env.last_offset)
            {
                return 0;
            }
            else
            {
                if( (r_len + speaker_env.last_read_offset) > sbc_sotre_env.last_offset  )
                {
                    flash_read(sbc_sotre_env.start_base + speaker_env.last_read_page_idx * FLASH_PAGE_SIZE + speaker_env.last_read_offset
                               ,sbc_sotre_env.last_offset - speaker_env.last_read_offset, sbc_buff + pos);
                    uint32_t no_read_len = ( r_len + speaker_env.last_read_offset - sbc_sotre_env.last_offset );
                    pos += r_len;
                    speaker_env.last_read_offset = sbc_sotre_env.last_offset;
                    r_len = 0;
                    return (read_len - no_read_len);
                }
                else
                {
                    flash_read(sbc_sotre_env.start_base + speaker_env.last_read_page_idx * FLASH_PAGE_SIZE + speaker_env.last_read_offset
                               ,r_len, sbc_buff + pos);
                    pos += r_len;
                    speaker_env.last_read_offset += r_len;
                    r_len = 0;
                }
            }
        }
        else
            return 0;
    }

    return read_len;
}


/*********************************************************************
 * @fn		decoder_start
 *
 * @brief	Start decoding
 *
 * @param	start           - starting address
 *          start           - end address
 *          tot_data_len    - Total length
 *          frame_len       - frame length
 *			start_offset    - Starting offset
 *          type            - type of data (RAM or Flash)
 * @return	None.
 */

//void decoder_start(uint32_t start, uint32_t end, uint32_t tot_data_len, uint16_t frame_len, uint32_t start_offset, uint8_t type)
//{
//    struct decoder_prepare_t param;

//    param.data_start = start;
//    param.data_end = end;
//    param.store_type = type;
//    param.tot_data_len = tot_data_len;
//    param.start_offset = start_offset;
//    param.frame_len = frame_len;
//    DEC_LOG("s:%x,e:%x\r\n",start,end);

//	os_event_t audio_event;

//	audio_event.event_id = DECODER_EVENT_PREPARE;
//	audio_event.param = &param;
//	audio_event.param_len = sizeof(param);
//    os_msg_post(audio_task_id,&audio_event);

//    speaker_start_hw();
//    decoder_hold_flag = false;
//}

/*********************************************************************
 * @fn		decoder_play_next_frame
 *
 * @brief	Release semaphore,Decode the next frame of data
 *
 * @param	None
 *         
 * @return	None.
 */
//void decoder_play_next_frame(void)
//{
//   

//	os_event_t audio_event;

//	audio_event.event_id = DECODER_EVENT_NEXT_FRAME;
//	audio_event.param = NULL;
//	audio_event.param_len = 0;
//    os_msg_post(audio_task_id,&audio_event);
//	DEC_LOG("decoder_play_next_frame\r\n");
//}

/*********************************************************************
 * @fn		decoder_stop
 *
 * @brief	Stop decoding
 *
 * @param	None
 *         
 * @return	None.
 */
//void decoder_stop(void)
//{
//    DEC_LOG("decoder_stop\r\n");
//    //printf("d_s\r\n");


//	os_event_t audio_event;

//	audio_event.event_id = DECODER_EVENT_STOP;
//	audio_event.param = NULL;
//	audio_event.param_len = 0;
//   os_msg_post(audio_task_id,&audio_event);
//}

/*********************************************************************
 * @fn		test_end_speaker
 *
 * @brief	 stop audio 
 *
 * @param	None
 *         
 * @return	None.
 */
//void test_end_speaker(void)
//{
//    decoder_stop();
//    speaker_stop_hw();
//}


/*********************************************************************
 * @fn		test_speaker_from_flash
 *
 * @brief	Playing audio from flash
 *
 * @param	None
 *         
 * @return	None.
 */
//void test_speaker_from_flash(void)
//{

//    if( sbc_buff != NULL)
//        goto _Exit;
//    co_printf("speaker_flash_start\r\n");

//    memset((void *)&sbc_sotre_env, 0, sizeof(sbc_sotre_env));
//    flash_read(USER_FLASH_BASE_ADDR, sizeof(sbc_sotre_env), (uint8_t *)&sbc_sotre_env);//读取Flash存储的audio信息
//    co_printf("%x,%d,%d\r\n",sbc_sotre_env.start_base,sbc_sotre_env.last_page_idx,sbc_sotre_env.last_offset);
//    if(sbc_sotre_env.start_base == 0xffffffff)//异常
//    {
//        memset((void *)&sbc_sotre_env,0x00,sizeof(sbc_sotre_env));
//		Flash_data_state = false;//flash没有音频数据
//        goto _Exit;
//    }
//	Flash_data_state = true;//flash中有音频数据
//    speaker_init();//speaker 初始化

//    sbc_buff = (uint8_t *)os_zalloc(10*1024);//申请10kbuffer


//    uint8_t *tmp_buf;

//    flash_read(sbc_sotre_env.start_base, 512, sbc_buff);//读取512个字节的数据

//    memset((void *)&speaker_env, 0, sizeof(speaker_env));
//    tmp_buf = sbc_buff;
//    speaker_env.sbc_frame_len = decoder_calc_adpcm_ms_frame_len(&tmp_buf);//获取sbc_frame_len
//    //sbc_sotre_env.start_base += (tmp_buf - sbc_buff);
//    speaker_env.last_read_offset = (tmp_buf - sbc_buff);
//    speaker_env.last_read_page_idx = 0;
//    if(speaker_env.sbc_data_tot_len == 0)
//        speaker_env.sbc_data_tot_len = (10240 - 10240%speaker_env.sbc_frame_len)&(~0x1);
//    speaker_env.store_data_len += read_sbc_from_flash(sbc_buff,speaker_env.sbc_data_tot_len>>1);

//    decoder_start((uint32_t)sbc_buff, (uint32_t)sbc_buff + speaker_env.sbc_data_tot_len
//                  , speaker_env.store_data_len, speaker_env.sbc_frame_len, 0, DECODER_STORE_TYPE_RAM);

//_Exit:
//    ;
//}

/*********************************************************************
 * @fn		decoder_update_tot_data_len
 *
 * @brief	Update total data length
 *
 * @param	len    -    Latest total data length
 *         
 * @return	None.
 */

void decoder_update_tot_data_len(uint32_t len)
{
    decoder_env.tot_data_len = len;
}

/*********************************************************************
 * @fn		decoder_half_processed
 *
 * @brief	Put new playback data into the cache and update the total audio data length tot_data_len.
 *
 * @param	None
 *		   
 * @return	None.
 */
void decoder_half_processed(void)
{
    if(speaker_env.end_flag)
        goto _Exit;
    else
    {
        uint32_t pos = (speaker_env.store_data_len % speaker_env.sbc_data_tot_len);
        //fputc('z',0);
        uint32_t read_len = read_sbc_from_flash(sbc_buff + pos,speaker_env.sbc_data_tot_len>>1);
        if(read_len > 0)
        {
            speaker_env.store_data_len += read_len;
            decoder_update_tot_data_len( speaker_env.store_data_len );
        }
        else
            speaker_env.end_flag = 1;
    }
_Exit:
    ;
}

/*********************************************************************
 * @fn		decoder_play_next_frame_handler
 *
 * @brief	Processing and decoding the next frame of audio data,
 *
 * @param	arg    -    task state
 *         
 * @return	None.
 */

//void  decoder_play_next_frame_handler(void *arg)
//{
//    uint8_t *buffer;
//    uint32_t pcm_len;
//    uint32_t streamlen;
//    struct decoder_pcm_t *pcm_frame;
//	uint8_t *Task_state;
//	Task_state = arg;
//    // CPU_SR cpu_sr;

//    switch(*Task_state)
//    {
//        case DECODER_STATE_IDLE:
//            break;
//        case DECODER_STATE_BUFFERING:
//        case DECODER_STATE_PLAYING:
//#if 1

//            if(decoder_env.store_type == DECODER_STORE_TYPE_RAM)
//            {
//                buffer = (uint8_t *)decoder_env.current_pos;
//            }
//            else
//            {
//                // TBD
//            }

//            pcm_frame = (struct decoder_pcm_t *)os_zalloc(sizeof(struct decoder_pcm_t) + 2*(decoder_env.frame_len+8)*2);
//            if(pcm_frame == NULL)
//            {
//               // return KE_MSG_SAVED;
//            }

//            pcm_len = 2*(decoder_env.frame_len+8)*2;
//            streamlen = decoder_env.frame_len;
//            DEC_LOG("playing:%d\r\n",streamlen);
//            //printf("p:%d\r\n",streamlen);
//            DEC_LOG("sbc_buff[0] = %02x, %02x.\r\n", buffer[0], buffer[1]);

//            if(decoder_hold_flag == false)
//            {
//                adpcm_decode_frame(decoder_env.decoder_context, (short *)&pcm_frame->pcm_data[0], (int *)&pcm_len, buffer, decoder_env.frame_len);
//            }
//            else
//                memset((uint8_t *)&pcm_frame->pcm_data[0],0x0,pcm_len);

//            if( 1 )
//            {
//                pcm_frame->pcm_size = pcm_len >> 1;
//                pcm_frame->pcm_offset = 0;
//                GLOBAL_INT_DISABLE();
//                co_list_push_back(&decoder_env.pcm_buffer_list,&pcm_frame->list);
//                GLOBAL_INT_RESTORE();
//                DEC_LOG("pcmlen=%d,%d\r\n",pcm_len,streamlen);
//                decoder_env.pcm_buffer_counter++;

//                if((*Task_state )== DECODER_STATE_BUFFERING)
//                {
//					if (decoder_env.pcm_buffer_counter > 2)
//                    {
//                 		decodeTASKState = DECODER_STATE_PLAYING;
//                        NVIC_EnableIRQ(I2S_IRQn);//
//						
//                    }
//                    else
//                    {
//						decoder_play_next_frame();
//						
//                    }
//                }

//                if(decoder_hold_flag == false)
//                {
//                    decoder_env.current_pos += decoder_env.frame_len;
//                    decoder_env.data_processed_len += decoder_env.frame_len;
//                    if( (decoder_env.tot_data_len - decoder_env.data_processed_len) < (1024) )
//                    {
//						decoder_half_processed();
//                    }

//                    if( (decoder_env.tot_data_len - decoder_env.data_processed_len)  < decoder_env.frame_len )
//                    {
//                        if(decoder_hold_flag == false)
//                        {                 
//                        	decodeTASKState = DECODER_STATE_WAITING_END;                          
//                        }
//                    }
//                    if(decoder_env.current_pos >= decoder_env.data_end)
//                    {
//                       decoder_env.current_pos = decoder_env.data_start;
//                    }
//                }

//                if(decoder_env.store_type == DECODER_STORE_TYPE_FLASH)
//                {
//                    // TBD, free buffer
//                }
//            }
//            else
//            {
//                os_free(pcm_frame);
//                NVIC_EnableIRQ(I2S_IRQn);
//                decoder_stop();
//            }
//#endif
//            break;
//        case DECODER_STATE_WAITING_END:
//            DEC_LOG("STATE_WAITING_END\r\n");
//            //if(decoder_env.pcm_buffer_list.first == NULL)
//            if(decoder_env.pcm_buffer_counter == 0)
//            {
//                if(stop_flag == 0)
//                {
//                    decoder_stop();
//                    stop_flag = 1;
//                }
//            }
//            break;
//        default:
//            break;
//    }

//}

/*********************************************************************
 * @fn		decoder_end_func
 *
 * @brief	Callback function after decoding，
 *
 * @param	None
 *		   
 * @return	None.
 */
void decoder_end_func(void)
{
    co_printf("E");
    if(sbc_buff!= NULL)
    {
        os_free(sbc_buff);
        sbc_buff = NULL;
    }

}



//有效音频数据解码ok
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
///////////ok
uint16_t get_audio_simplerate(void)
{
	return user_wav_fotmat_info.wav_format.fmt.SampleRate;
}
