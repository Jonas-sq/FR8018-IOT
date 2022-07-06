#include "stdint.h"
//#include "decoder.h"
#include "os_msg_q.h"
#include "co_printf.h"
#include "os_task.h"
//#include "driver_i2s.h"
#include "driver_pmu.h"
#include "driver_gpio.h"
//#include "driver_codec.h"
//#include "audio_play.h"
#include "driver_uart.h"
#include "ble_simple_peripheral.h"
#include <string.h>
#include "tongxin.h"
#include "os_timer.h"
#include "driver_timer.h"

#include "sys_utils.h"

uint16_t tongxin_task_id;

static volatile uint16_t cntRxd = 0;
uint8_t cmdArrived = 0; 
static uint8_t bufRxd[30];
uint16_t tongxin_uart_task_id;



unsigned char UartRead(unsigned char *buf, unsigned char len) //串口数据读取函数，数据接收指针buf，读取数据长度len，返回值为实际读取到的数据长度
{
    unsigned char i;
    
    if (len > cntRxd) //读取长度大于接收到的数据长度时，
    {  
        len = cntRxd; //读取长度设置为实际接收到的数据长度
    }
    for (i=0; i<len; i++) //拷贝接收到的数据
    {
        *buf = bufRxd[i];
        buf++;
    }
    cntRxd = 0;  //清零接收计数器
    
	
    return len;  //返回实际读取长度
}

//unsigned char GetYH(unsigned char *ptr,  unsigned char len)
//{ 
//	unsigned char crc=0,result=0;
//	unsigned char i;
//	crc = *(ptr+4);

//	for(i=5;i<len-3;i++)
//	{  
//	  result = crc ^ *(ptr+i);
//  	crc = result;
//	}

//	return crc;
//}


void UartRxMonitor(unsigned char ms)  //串口接收监控函数
{
  unsigned char len;
  unsigned char buf[30];
	unsigned int temp;
  unsigned char sudu[2];

   uint8_t test1[]={0xaa,0xfc,0x85,0x87,0x58,0x09,0x01,0x00,0x50,0x0a,0x0d};
	
	
    static unsigned char cntbkp = 0;
    static unsigned char idletmr = 0;
//uint8_t test[5]={0x01,0x02,0x03,0x04,0x05};
    if (cntRxd > 0)  //接收计数器大于零时，监控总线空闲时间
    {
        if (cntbkp != cntRxd)  //接收计数器改变，即刚接收到数据时，清零空闲计时
        { 
            cntbkp = cntRxd;
            idletmr = 0;
        }
        else
        {
            if (idletmr < 5)  //接收计数器未改变，即总线空闲时，累积空闲时间
            {
                idletmr += ms;
                if (idletmr >= 5)  //空闲时间超过4个字节传输时间即认为一帧命令接收完毕
                {
                    cmdArrived = 1; //设置命令到达标志

//										os_event_t tongxin_event;
//										uint8_t param = 40;
//										tongxin_event.event_id = 2;
//										tongxin_event.param = &param;
//										tongxin_event.param_len = sizeof(param);
//										tongxin_event.param = NULL;
//										tongxin_event.src_task_id = tongxin_task_id;
//										os_msg_post(tongxin_task_id,&tongxin_event);	
								
                }
            }
        }
    }
    else
    {
        cntbkp = 0;
    }
}

__attribute__((section("ram_code"))) void timer1_isr_ram(void)
{
  timer_reload(TIMER1);
	timer_clear_interrupt(TIMER1);

	UartRxMonitor(1);
}	

__attribute__((section("ram_code"))) void uart0_isr_ram(void)
{
    uint8_t int_id;
    //uint8_t c;
    volatile struct uart_reg_t *uart_reg = (volatile struct uart_reg_t *)UART0_BASE;

    int_id = uart_reg->u3.iir.int_id;

    if(int_id == 0x04 || int_id == 0x0c )   /* Receiver data available or Character time-out indication */
    {
      		while(uart_reg->lsr & 0x01) 
				{
					bufRxd[cntRxd++] = uart_reg->u1.data;										
      	} 	

    }
    else if(int_id == 0x06)
    {
        volatile uint32_t line_status = uart_reg->lsr;
    }
}



//void UartDriver()
//{
//	  unsigned char len;
//	  unsigned char data_len;
//	  unsigned char crc;
//    unsigned char buf[30];
//   if (cmdArrived) //有命令到达时，读取处理该命令
//    {
//			 len = UartRead(buf, sizeof(buf)); //将接收到的命令读取到缓冲区中
//			 cmdArrived = 0;
//	/********************************************************************************/			
//			 if ((buf[0] == 0xAA) && (buf[1] == 0xFC) && (buf[2] == 0x85)&& (buf[3] == 0x86)) //核对地址以决定是否响应命令，本例中的本机地址为0x01
//        {
//				  crc=GetYH(buf,len);
//					if(buf[len-3] == crc)
//					{	
//					       switch (buf[4]) //按功能码执行操作
//                {
//                   case 0x55:
//												buf[4]=0x59;
//									      break;
//																							 									 									 									 									 
//									 case 0x56:  //读取一个或连续的寄存器
//                        break;
//									 
//                   case 0x58:  //写入单个寄存器
//										    data_len = buf[6];//数据长度
//									 
//									 
//										 		buf[4]=0x61;
//												break;
//								}
//								  buf[len-3]=GetYH(buf,len);
//									uart_write(UART0,buf,len); 
//					}
//				}
//		}
//		co_delay_100us(1000);
//		gpio_portb_write(gpio_portb_read() 	& ~ (1<<GPIO_BIT_5) );//0
//}


//static int tongxin_task_func(os_event_t *param)
//{
//		switch(param->event_id)
//	{
//		case 2:
//		{
//					gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_5) );//1
//					UartDriver();
////			 //co_printf("Task test!\r\n");
////         uint8_t test[5]={0x01,0x02,0x03,0x04,0x06};	
////				 		  
////	       			
//		}
//		break;
//		
//	}
//		
//	return EVT_CONSUMED;
//	
//}

//void  tongxin_task_init(void)
//{ 
// tongxin_task_id = os_task_create(tongxin_task_func);
//}


