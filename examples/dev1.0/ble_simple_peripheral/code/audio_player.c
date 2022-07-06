#include "audio_player.h"
#include "driver_i2s.h"
#include "driver_codec.h"
#include "driver_pmu.h"
#include "driver_gpio.h"
#include "audio_decode.h"
#include "co_printf.h"
#include "os_task.h"
#include "os_msg_q.h"

uint16_t audio_task_id;
uint8_t audio_data_read_finish;
void PA_init_pins(void)
{
	system_set_port_mux(GPIO_PORT_D, GPIO_BIT_7, PORTD7_FUNC_D7);
	gpio_set_dir(GPIO_PORT_D, GPIO_BIT_7, GPIO_DIR_OUT);
	gpio_portd_write(gpio_portd_read() & ~(1<<GPIO_BIT_7) );
}

void speaker_start(void)
{
	pmu_codec_power_enable();
	audio_speaker_codec_init();	
	i2s_init(I2S_DIR_TX,get_audio_simplerate(),1);	
	PA_init_pins();		              
	i2s_start();                            
    codec_enable_dac();                     
	
	gpio_portd_write(gpio_portd_read() | (1<<GPIO_BIT_7) );
	NVIC_EnableIRQ(I2S_IRQn);	
	NVIC_SetPriority(I2S_IRQn, 3);
}

void speacker_stop(void)
{
	NVIC_DisableIRQ(I2S_IRQn);      
	gpio_portd_write(gpio_portd_read() & ~(1<<GPIO_BIT_7) );
	codec_disable_dac();				
    i2s_stop();							
	pmu_codec_power_disable(); 	
}

__attribute__((section("ram_code"))) void i2s_isr_ram(void)
{
     uint16_t data[I2S_FIFO_DEPTH/2];
	uint32_t len;
	if((i2s_reg->status.tx_half_empty)&&(i2s_reg->mask.tx_half_empty))
	{       
		if(lwrb_get_full(&lwrb_audio_header) == 0 && (audio_data_read_finish == 1))
		{
			memset(data, 0, sizeof(data));
			for (uint8_t i=0; i<(I2S_FIFO_DEPTH/2); i++)
			{
				uint32_t tmp_data = data[i];
				tmp_data &= 0xFFFF;
				i2s_reg->data = tmp_data;
			}
			audio_task_msg_post(AUDIO_PLAY_STOP);
		}
		else
		{
			len = lwrb_get_full(&lwrb_audio_header);
			if( len < 2048)
			{
				audio_task_msg_post(AUDIO_PLAY_NEXT_FRAME);
			}
			lwrb_read(&lwrb_audio_header, data, sizeof(data));
			for (uint8_t i=0; i<(I2S_FIFO_DEPTH/2); i++)
			{
				uint32_t tmp_data = data[i];
				tmp_data &= 0xFFFF;
				i2s_reg->data = tmp_data;
			}
		}
	}		  
}



//从flash读取音频文件信息准备播放
uint8_t audio_play_ready(uint32_t flash_addr)  
{
	uint8_t result = 0;
	result = audio_wave_header(flash_addr);
	//memset(audio_decode_data_block,0, sizeof(audio_decode_data_block));
	if(!result)  //文件信息正确
	{
		audio_task_msg_post(AUDIO_PLAY_START);
		return 0;
	}
	return 1;
}

//音频播放一帧
void audio_paly_frame(void)
{
	audio_data_read_finish = audio_data_decode();
}

//音频播放
void audio_play_start(void)
{
	audio_paly_frame();
	speaker_start();	
	co_printf("audio start\r\n");
}

//音频停止
void audio_play_stop(void)
{
	speacker_stop();
	lwrb_reset(&lwrb_audio_header);
	co_printf("audio stop\r\n");
}


void audio_task_msg_post(uint8_t event_id)
{
	os_event_t audio_event;

	audio_event.event_id = event_id;
	audio_event.param = NULL;
	audio_event.param_len = 0;
    os_msg_post(audio_task_id,&audio_event);
}

static int audio_task_func(os_event_t *param)
{
	switch(param->event_id)
	{
		case AUDIO_PLAY_START:
			audio_play_start();
			audio_data_read_finish = 0;		
		break;;
		
		case AUDIO_PLAY_NEXT_FRAME:
			 audio_paly_frame();
		break;;
		
		case AUDIO_PLAY_STOP:
			audio_play_stop();
			audio_data_read_finish = 0;
		break;;
	}
	return EVT_CONSUMED;
}

void audio_task_init(void)
{
	 audio_task_id = os_task_create(audio_task_func);
}

