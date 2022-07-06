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
#include "speaker.h"
#include "driver_flash.h"
#include "driver_uart.h"
#include "co_printf.h"
#include "os_task.h"
#include "os_timer.h"
#include "sys_utils.h"
#include "lwrb.h"
#include "driver_gpio.h"
#include "driver_timer.h"


#define UART_RX_BUFFER_MAX   300
#define UART_RX_DATA_LEN_MAX 256 + 4
static volatile uart_rev_state_type_t current_uart_rev_state_type = UART_REV_STATE_FOUND_NULL;
static uint8_t UART_RX_Buffer[UART_RX_BUFFER_MAX];
static uint8_t UART_RX_Buffer_temp[2] = {0};
static uint16_t uart_data_len =  0;
static volatile uint16_t UART_RX_Count = 0;
uint8_t lwrb_uart_buff[UART_RX_BUFFER_MAX * 2];
uint8_t read_buff[UART_RX_BUFFER_MAX];
uint16_t audio_uart_task_id;
lwrb_t lwrb_uart_header; 


/*
 * MACROS
 */
#define PA_ENABLE      gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_4) )//gpio_set_pin_value(GPIO_PORT_A, GPIO_BIT_1, 1)
#define PA_DISABLE     gpio_portb_write(gpio_portb_read() & ~(1<<GPIO_BIT_4) )//gpio_set_pin_value(GPIO_PORT_A, GPIO_BIT_1, 0)

//uint8_t TimerCnt=0;


static void audio_flash_erase_page(uint32_t addr)  //擦除一页
{
//	W25QXX_Erase_Sector(addr);
	flash_erase(addr, 0x1000);
}

/////OK
static void audio_flash_write(uint32_t addr, uint8_t *buff, uint32_t length)
{
//	W25QXX_Write(buff,addr,length);
	flash_write(addr, length, buff);
}

//void audio_uart_task_msg_post(uint8_t event_id)
//{
//	os_event_t audio_event;

//	audio_event.event_id = event_id;
//	audio_event.param = NULL;
//	audio_event.param_len = 0;
//  os_msg_post(audio_uart_task_id,&audio_event);
//}


void erase_flash_ack(void)
{
	uint8_t buff[] = "erase ok\r\n";
	for(uint8_t i = 0; i < sizeof(buff); i++)
	{
		uart_putc_noint_no_wait(UART1, buff[i]);
	}
	
}


void write_flash_ack(void)
{
	uint8_t buff[] = "write ok\r\n";
	for(uint8_t i = 0; i < sizeof(buff); i++)
	{
		uart_putc_noint_no_wait(UART1, buff[i]);
	}
	
}

/*********************************************************************
 * @fn      audio_task_func
 *
 * @brief   Audio task function, handles audio events.
 *
 * @param   param   - OS events of audio.
 *       
 *
 * @return  int     - EVT_CONSUMED.
 */

static int audio_task_func(os_event_t *param)
{
	uint32_t flash_addr = 0;
	uint16_t len = 0;
	switch(param->event_id)
	{
		
		case 1:
				lwrb_read(&lwrb_uart_header, &read_buff, param->param_len);
		         
		      
		        len = read_buff[3] << 8 | read_buff[4];
				 
		        if((read_buff[0] == 0x55) && (read_buff[1] == 0xaa) && (len == param->param_len - 5))   //帧头和长度正确
				{
					if(read_buff[2] == 0x01)  //擦除指令
					{
						flash_addr = (uint32_t) ((read_buff[5] << 24) | (read_buff[6] << 16) | (read_buff[7] << 8) | (read_buff[8] << 0));
						audio_flash_erase_page(flash_addr);
						erase_flash_ack();
					}
					else if(read_buff[2] == 0x02) //写指令  
					{
						flash_addr = (uint32_t) ((read_buff[5] << 24) | (read_buff[6] << 16) | (read_buff[7] << 8) | (read_buff[8] << 0));
						audio_flash_write(flash_addr, &read_buff[9], len - 4);
					//	co_printf("write ok\r\n");
						write_flash_ack();
					}
				}
			break;
		case 2:
				
		break;
				
		case 3:
				
			break;
	}
	return EVT_CONSUMED;
}



void audio_uart_task_init(void)
{
	 audio_uart_task_id = os_task_create(audio_task_func);
}

void audio_uart_init(void)
{
	system_set_port_pull(GPIO_PA2, true);
  system_set_port_mux(GPIO_PORT_A, GPIO_BIT_2, PORTA2_FUNC_UART1_RXD);
  system_set_port_mux(GPIO_PORT_A, GPIO_BIT_3, PORTA3_FUNC_UART1_TXD);
	NVIC_SetPriority(UART1_IRQn, 2);
	NVIC_EnableIRQ(UART1_IRQn);
  uart_init(UART1, BAUD_RATE_115200);  
	
	
	lwrb_init(&lwrb_uart_header, lwrb_uart_buff, sizeof(lwrb_uart_buff));
	lwrb_reset(&lwrb_uart_header);
	audio_uart_task_init();

}

static bool common_uart_data_unpack(uint8_t data)
{
    bool ret = false;

    UART_RX_Buffer_temp[0] = UART_RX_Buffer_temp[1];
    UART_RX_Buffer_temp[1] = data;

    if(((UART_RX_Buffer_temp[0]==0x55))&&(UART_RX_Buffer_temp[1]==0xAA))
    {
        memset(UART_RX_Buffer,0,sizeof(UART_RX_Buffer));
        memcpy(UART_RX_Buffer,UART_RX_Buffer_temp,2);
        memset(UART_RX_Buffer_temp,0,2);
        UART_RX_Count = 2;
        current_uart_rev_state_type = UART_REV_STATE_FOUND_HEAD;
        uart_data_len = 0;
        return ret;
    }

    switch(current_uart_rev_state_type)
    {
		case UART_REV_STATE_FOUND_NULL:
			break;
		case UART_REV_STATE_FOUND_HEAD:
			UART_RX_Buffer[UART_RX_Count++] = data;
			current_uart_rev_state_type = UART_REV_STATE_FOUND_CMD;
			break;
		case UART_REV_STATE_FOUND_CMD:
			UART_RX_Buffer[UART_RX_Count++] = data;
			current_uart_rev_state_type = UART_REV_STATE_FOUND_LEN_H;
			break;
		case UART_REV_STATE_FOUND_LEN_H:
			UART_RX_Buffer[UART_RX_Count++] = data;
			uart_data_len = (UART_RX_Buffer[UART_RX_Count-2]<<8)|UART_RX_Buffer[UART_RX_Count-1];
		    
			if(uart_data_len>UART_RX_DATA_LEN_MAX)
			{
				
				memset(UART_RX_Buffer_temp,0,2);
				memset(UART_RX_Buffer,0,sizeof(UART_RX_Buffer));
				UART_RX_Count = 0;
				current_uart_rev_state_type = UART_REV_STATE_FOUND_NULL;
				uart_data_len = 0;
			}
			else if(uart_data_len>0)
			{
				current_uart_rev_state_type = UART_REV_STATE_FOUND_LEN_L;
			}
			else
			{
				current_uart_rev_state_type = UART_REV_STATE_FOUND_DATA;
			}
			break;
		case UART_REV_STATE_FOUND_LEN_L:
			UART_RX_Buffer[UART_RX_Count++] = data;   //DATA
			uart_data_len--;
			
			if(uart_data_len==0)
			{
				lwrb_write(&lwrb_uart_header, &UART_RX_Buffer, UART_RX_Count);
				os_event_t audio_event;
				audio_event.event_id = 1;
				audio_event.param = NULL;
				audio_event.param_len = UART_RX_Count;
			    os_msg_post(audio_uart_task_id,&audio_event);
				 memset(UART_RX_Buffer_temp,0,3);
				memset(UART_RX_Buffer,0,sizeof(UART_RX_Buffer));
				UART_RX_Count = 0;
				current_uart_rev_state_type = UART_REV_STATE_FOUND_NULL;
				uart_data_len = 0;
				ret = true;
			  //  current_uart_rev_state_type = UART_REV_STATE_FOUND_DATA;
			}
			break;
		case UART_REV_STATE_FOUND_DATA:
			UART_RX_Buffer[UART_RX_Count++] = data;  //sum data
			ret = true;
			break;
		default:
			memset(UART_RX_Buffer_temp,0,3);
			memset(UART_RX_Buffer,0,sizeof(UART_RX_Buffer));
			UART_RX_Count = 0;
			current_uart_rev_state_type = UART_REV_STATE_FOUND_NULL;
			uart_data_len = 0;
			break;
    };

    return ret;

}

__attribute__((section("ram_code"))) void uart1_isr_ram(void)
{//co_printf("xxxxxxxxxxxxxxxxxxxxxxx \r\n");
    uint8_t int_id;
    uint8_t c;
    volatile struct uart_reg_t *uart_reg = (volatile struct uart_reg_t *)UART1_BASE;

    int_id = uart_reg->u3.iir.int_id;
    if(int_id == 0x04 || int_id == 0x0c )   /* Receiver data available or Character time-out indication */
    {
		while(uart_reg->lsr & 0x01) 
		{
		  c = uart_reg->u1.data;
			//uart_putc_noint(UART1,c);
		}
      
		common_uart_data_unpack(c);
    }
    else if(int_id == 0x06)
    {
        volatile uint32_t line_status = uart_reg->lsr;
    }
}

