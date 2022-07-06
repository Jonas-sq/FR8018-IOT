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



unsigned char UartRead(unsigned char *buf, unsigned char len) //�������ݶ�ȡ���������ݽ���ָ��buf����ȡ���ݳ���len������ֵΪʵ�ʶ�ȡ�������ݳ���
{
    unsigned char i;
    
    if (len > cntRxd) //��ȡ���ȴ��ڽ��յ������ݳ���ʱ��
    {  
        len = cntRxd; //��ȡ��������Ϊʵ�ʽ��յ������ݳ���
    }
    for (i=0; i<len; i++) //�������յ�������
    {
        *buf = bufRxd[i];
        buf++;
    }
    cntRxd = 0;  //������ռ�����
    
	
    return len;  //����ʵ�ʶ�ȡ����
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


void UartRxMonitor(unsigned char ms)  //���ڽ��ռ�غ���
{
  unsigned char len;
  unsigned char buf[30];
	unsigned int temp;
  unsigned char sudu[2];

   uint8_t test1[]={0xaa,0xfc,0x85,0x87,0x58,0x09,0x01,0x00,0x50,0x0a,0x0d};
	
	
    static unsigned char cntbkp = 0;
    static unsigned char idletmr = 0;
//uint8_t test[5]={0x01,0x02,0x03,0x04,0x05};
    if (cntRxd > 0)  //���ռ�����������ʱ��������߿���ʱ��
    {
        if (cntbkp != cntRxd)  //���ռ������ı䣬���ս��յ�����ʱ��������м�ʱ
        { 
            cntbkp = cntRxd;
            idletmr = 0;
        }
        else
        {
            if (idletmr < 5)  //���ռ�����δ�ı䣬�����߿���ʱ���ۻ�����ʱ��
            {
                idletmr += ms;
                if (idletmr >= 5)  //����ʱ�䳬��4���ֽڴ���ʱ�伴��Ϊһ֡����������
                {
                    cmdArrived = 1; //����������־

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
//   if (cmdArrived) //�������ʱ����ȡ���������
//    {
//			 len = UartRead(buf, sizeof(buf)); //�����յ��������ȡ����������
//			 cmdArrived = 0;
//	/********************************************************************************/			
//			 if ((buf[0] == 0xAA) && (buf[1] == 0xFC) && (buf[2] == 0x85)&& (buf[3] == 0x86)) //�˶Ե�ַ�Ծ����Ƿ���Ӧ��������еı�����ַΪ0x01
//        {
//				  crc=GetYH(buf,len);
//					if(buf[len-3] == crc)
//					{	
//					       switch (buf[4]) //��������ִ�в���
//                {
//                   case 0x55:
//												buf[4]=0x59;
//									      break;
//																							 									 									 									 									 
//									 case 0x56:  //��ȡһ���������ļĴ���
//                        break;
//									 
//                   case 0x58:  //д�뵥���Ĵ���
//										    data_len = buf[6];//���ݳ���
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


