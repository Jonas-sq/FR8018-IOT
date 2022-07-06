/**
 * Copyright (c) 2019, Freqchip
 * 
 * All rights reserved.
 * 
 * 
 */
 
/*
 * INCLUDES (包含头文件)
 */
#include <stdbool.h>
#include "gap_api.h"
#include "gatt_api.h"
#include "driver_gpio.h"
#include "driver_pmu.h"
#include "button.h"
#include "os_timer.h"
#include "speaker_service.h"
#include "simple_gatt_service.h"
#include "ble_simple_peripheral.h"

#include "sys_utils.h"
#include "lcd.h"
#include "capb18-001.h"
#include "sht3x.h"
#include "decoder.h"
#include "gyro_alg.h"
////////////////////////////////////TIMER
#include <stdint.h>
#include "co_printf.h"
#include "driver_system.h"
#include "driver_timer.h"

#include "driver_uart.h"
#include "tongxin.h"

#define PB_ENABLE      gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_4) )
#define PB_DISABLE     gpio_portb_write(gpio_portb_read() & ~(1<<GPIO_BIT_4) )
unsigned int Rf_Cnt=0;
unsigned int Lo_Cnt;
unsigned int Hi_Cnt;

unsigned int Count_Lead;
unsigned int Count_Data_Hi;
unsigned int Count_Data_Lo;

unsigned int Recv_Bit_Cnt;
unsigned int Recv_Byte_Cnt;
unsigned int Recv_Data_Buf;
unsigned int Rf_Data[3];
unsigned int Rf_Control_Data;
uint8_t laba;


//uint8_t test[30]={0xAA,0xFC,0x85,0x87,0x058,0x09,0x01,0x00};

unsigned char UartRead(unsigned char *buf, unsigned char len);

uint8_t sp_conidx;//蓝牙连接号conidx

/*
 * MACROS
 */

/*
 * CONSTANTS 
 */
const unsigned	char * lcd_show_workmode[MODE_MAX] = {"PICTURE_UPDATE","SENSOR_DATA","SPEAKER_FROM_FLASH","CODEC_TEST"};
void audio_play_ready(uint32_t flash_addr);
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
// GAP-广播包的内容,最长31个字节.短一点的内容可以节省广播时的系统功耗.
static uint8_t adv_data[] =
{
  // service UUID, to notify central devices what services are included
  // in this peripheral. 告诉central本机有什么服务, 但这里先只放一个主要的.
  0x03,   // length of this data
  GAP_ADVTYPE_16BIT_MORE,      // some of the UUID's, but not all
  0xFF,
  0xFE,
};

// GAP - Scan response data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
// GAP-Scan response内容,最长31个字节.短一点的内容可以节省广播时的系统功耗.
static uint8_t scan_rsp_data[] =
{
  // complete name 设备名字
  0x12,   // length of this data
  GAP_ADVTYPE_LOCAL_NAME_COMPLETE,
  'B','L','E',' ','T','E','S','T',' ','J','i','n',' ','P','e','n','g',

  // Tx power level 发射功率
  0x02,   // length of this data
  GAP_ADVTYPE_POWER_LEVEL,
  0,	   // 0dBm
};

/*
 * TYPEDEFS 
 */

/*
 * GLOBAL VARIABLES 
 */

os_timer_t timer_refresh;// 用于刷新传感器数据以及显示等
uint8_t App_Mode = SPEAKER_FROM_FLASH;//PICTURE_UPDATE;//工作模式  可以通过KEY1切换

/*
 * LOCAL VARIABLES 
 */
 
/*
 * LOCAL FUNCTIONS
 */
static void sp_start_adv(void);

/*
 * EXTERN FUNCTIONS
 */
uint8_t CAPB18_data_get(float *temperature,float *air_press);
uint8_t demo_CAPB18_APP(void);

/*
 * PUBLIC FUNCTIONS
 */

/** @function group ble peripheral device APIs (ble外设相关的API)
 * @{
 */

/*********************************************************************
 * @fn      app_gap_evt_cb
 *
 * @brief   Application layer GAP event callback function. Handles GAP evnets.
 *
 * @param   p_event - GAP events from BLE stack.
 *       
 *
 * @return  None.
 */
void app_gap_evt_cb(gap_event_t *p_event)
{
    switch(p_event->type)
    {
        case GAP_EVT_ADV_END:
        {
            co_printf("adv_end,status:0x%02x\r\n",p_event->param.adv_end_status);
            //gap_start_advertising(0);
        }
        break;
        
        case GAP_EVT_ALL_SVC_ADDED:
        {
            co_printf("All service added\r\n");
            sp_start_adv();
#ifdef USER_MEM_API_ENABLE
            //show_mem_list();
            //show_msg_list();
            //show_ke_malloc();
#endif
        }
        break;

        case GAP_EVT_SLAVE_CONNECT:
        {
					  sp_conidx = p_event->param.slave_connect.conidx;
            co_printf("slave[%d],connect. link_num:%d\r\n",p_event->param.slave_connect.conidx,gap_get_connect_num());
						gatt_mtu_exchange_req(p_event->param.slave_connect.conidx);
            gap_conn_param_update(p_event->param.slave_connect.conidx, 6, 6, 0, 500);
					 // gap_security_send_pairing_password(p_event->param.slave_connect.conidx,123456);
					//gap_security_req(p_event->param.slave_connect.conidx);/////////////////////
					
        }
        break;

        case GAP_EVT_DISCONNECT:
        {
            co_printf("Link[%d] disconnect,reason:0x%02X\r\n",p_event->param.disconnect.conidx
                      ,p_event->param.disconnect.reason);
            sp_start_adv();
#ifdef USER_MEM_API_ENABLE
            show_mem_list();
            //show_msg_list();
            show_ke_malloc();
#endif
        }
        break;

        case GAP_EVT_LINK_PARAM_REJECT:
            co_printf("Link[%d]param reject,status:0x%02x\r\n"
                      ,p_event->param.link_reject.conidx,p_event->param.link_reject.status);
            break;

        case GAP_EVT_LINK_PARAM_UPDATE:
            co_printf("Link[%d]param update,interval:%d,latency:%d,timeout:%d\r\n",p_event->param.link_update.conidx
                      ,p_event->param.link_update.con_interval,p_event->param.link_update.con_latency,p_event->param.link_update.sup_to);
            break;

        case GAP_EVT_PEER_FEATURE:
            co_printf("peer[%d] feats ind\r\n",p_event->param.peer_feature.conidx);
            show_reg((uint8_t *)&(p_event->param.peer_feature.features),8,1);
            break;

        case GAP_EVT_MTU:
            co_printf("mtu update,conidx=%d,mtu=%d\r\n"
                      ,p_event->param.mtu.conidx,p_event->param.mtu.value);
            break;
        
        case GAP_EVT_LINK_RSSI:
            co_printf("link rssi %d\r\n",p_event->param.link_rssi);
            break;
                
        case GAP_SEC_EVT_SLAVE_ENCRYPT:
            co_printf("slave[%d]_encrypted\r\n",p_event->param.slave_encrypt_conidx);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      sp_start_adv
 *
 * @brief   Set advertising data & scan response & advertising parameters and start advertising
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
static void sp_start_adv(void)
{
    // Set advertising parameters
    gap_adv_param_t adv_param;
    adv_param.adv_mode = GAP_ADV_MODE_UNDIRECT;
    adv_param.adv_addr_type = GAP_ADDR_TYPE_PUBLIC;
    adv_param.adv_chnl_map = GAP_ADV_CHAN_ALL;
    adv_param.adv_filt_policy = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    adv_param.adv_intv_min = 300;
    adv_param.adv_intv_max = 300;
        
    gap_set_advertising_param(&adv_param);
    
    // Set advertising data & scan response data
	gap_set_advertising_data(adv_data, sizeof(adv_data));
	gap_set_advertising_rsp_data(scan_rsp_data, sizeof(scan_rsp_data));
    // Start advertising
	co_printf("Start advertising...\r\n");
	gap_start_advertising(0);
}

unsigned char GetYH(unsigned char *ptr,  unsigned char len)
{ 
	unsigned char crc=0,result=0;
	unsigned char i;
	crc = *(ptr+4);

	for(i=5;i<len-3;i++)
	{  
	  result = crc ^ *(ptr+i);
  	crc = result;
	}

	return crc;
}


/*********************************************************************
 * @fn      timer_refresh_fun
 *
 * @brief   timer_refresh callback function
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void timer_refresh_fun(void *arg)
{	 
	os_timer_stop(&timer_refresh);
  //unsigned char len;
  unsigned char buf[30];
	unsigned int temp;
  unsigned char sudu[2];
	 uint8_t test[]={0xaa,0xfc,0x85,0x86,0x56,0x13,0x02,0x47,0x0a,0x0d};
   uint8_t test1[]={0xaa,0xfc,0x85,0x87,0x58,0x09,0x01,0x00,0x50,0x0a,0x0d};
	 gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_5) );                   //1  RS485EN=1 使能
	 uart_write(UART0,test,10);
	 co_delay_100us(100);
	 gpio_portb_write(gpio_portb_read() 	& ~ (1<<GPIO_BIT_5) );             //0  RS485EN=0 失能
	 
   if (cmdArrived) //有命令到达时，读取处理该命令
    { 
				//len = UartRead(buf, sizeof(buf));
				temp = buf[7]*256 + buf[8];
				sudu[0] = (unsigned char )(temp * 0.01);
				gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_5) );        //1  
			  test1[7] = sudu[0];
			  test1[8] = GetYH(test1,11);
				co_delay_100us(300);
				uart_write(UART0,test1,11);
				cmdArrived = 0;
	 	 	 	 test_gatt_notify(sp_conidx,test1[7]);
		}


		
	 os_timer_start(&timer_refresh,400,1);
}

__attribute__((section("ram_code"))) void timer0_isr_ram(void)
{
   // uint8_t code_1527;
	    timer_reload(TIMER0);
		  timer_clear_interrupt(TIMER0);
    switch(Rf_Cnt)
    {
      case 0 :                                                           //12ms引导码
	          if(gpio_get_pin_value(GPIO_PORT_A,GPIO_BIT_7)==0)                                                  //低电平累计次数
		      {
		        Count_Lead++;
		      }
	          else                                                       //高电平判断范围
		      {
				if((Count_Lead >= 56) && (Count_Lead <= 120))           //5.6ms - 12ms
				   {
						//co_printf("Count_Lead = %d \r\n ",Count_Lead);
					  Count_Lead=0;
                      Recv_Data_Buf = 0x00;                              //初始化参数
					  Count_Data_Hi = 0;
					  Count_Data_Lo = 0;
				      Recv_Bit_Cnt  = 0;
				      Recv_Byte_Cnt = 0;
					  Rf_Cnt=1;  
						 
				   }
				else                                                     //范围不对退出
                   {
					  Count_Lead=0;
					  Rf_Cnt=0;
				   }
	          }	
      break;

      case 1 :	                                                         //数据位高电平部分判断
			  if(gpio_get_pin_value(GPIO_PORT_A,GPIO_BIT_7)==1)                                                  //高电平累计次数
			  {
			    Count_Data_Hi++;
			  }
			  else                                                       //低电平判断范围
			  {                       
				 if((Count_Data_Hi >= 1) && (Count_Data_Hi <= 24))       // 80us - 2.4ms
					  {		
          // co_printf("Count_Lead = %d \r\n ",Count_Data_Hi);							
						Hi_Cnt = Count_Data_Hi;                          //储存计数，判断0 / 1
						Count_Data_Hi = 0;
					    Rf_Cnt=2;	
							
					  }
				 else
				      {		
						Rf_Cnt=0;
					  }
		       	}	  	
      break;

      case 2:                                                            //数据位低电平部分判断
			 if(gpio_get_pin_value(GPIO_PORT_A,GPIO_BIT_7)==0)                                                   //低电平累计次数
			 {
			   Count_Data_Lo++;
			 }
			 else                                                        //高电平判断范围
             {
				 if((Count_Data_Lo >= 1) && (Count_Data_Lo <= 24))       // 80us - 2.4ms
					  { 
					    Lo_Cnt = Count_Data_Lo;                          //储存计数，判断0 / 1
                        Count_Data_Lo = 0;	   
					    Rf_Cnt=3;
					  }
				 else
					 {		
						Rf_Cnt=0;
					 }
             }	  	
      break;

  	  case 3 :		
			  Recv_Data_Buf <<= 1;                                      //数据移位
			  if(Hi_Cnt>Lo_Cnt)                                         //0跟1区分，判断高低电平哪个长
			   {
				 Recv_Data_Buf|=0x01;
			   }
			  else
			   {
				 Recv_Data_Buf&=0xFE;
			   }
			   Recv_Bit_Cnt ++;	

			  if(Recv_Bit_Cnt>7)                                        //每8bit整理出一个byte
			   {
			     Rf_Data[Recv_Byte_Cnt]=Recv_Data_Buf;                  //存到数组里面
				 Recv_Bit_Cnt = 0;
				 Recv_Byte_Cnt++;
				 Recv_Data_Buf = 0x00;
			   }

			   if(Recv_Byte_Cnt>2)                                     //整理出4byte
			   {	
			     Rf_Control_Data=Rf_Data[2]&0x0F;                      //提取4bit控制数据
				 Rf_Cnt = 4;                                           //进入到功能码判断
			   }
			   else
			   {
				 Rf_Cnt = 1;                                           //不够3byte,继续去解码数据
			   }			   
      break;

      case 4 :                                                         //功能判断
				
              switch(Rf_Control_Data)
			  {
			    case 0x08 :
				//LED1=0;
					co_printf("LOCK \r\n ");
					laba=1;
					//PB_ENABLE;
					audio_play_ready(0x51000);
					gpio_portb_write(gpio_portb_read() & ~(1<<GPIO_BIT_3) );
                break;

				case 0x04 :
			//	LED1=1;
                break;

			    case 0x02 :
			//	LED2=0;
					co_printf("OPEN \r\n ");
					//PB_ENABLE;
					laba=0;
					audio_play_ready(0x51000);
				gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_3) );
				//	PB_DISABLE;
					//test_speaker_from_flash();
                break;

				case 0x01 :
			//	LED2=1;
                break;

				default:
				break;
			  }
	  Rf_Cnt = 0;                                                     //????????
      break;

      default:
	  Rf_Cnt = 0;
      break;
     }
	

}

/*********************************************************************
 * @fn      simple_peripheral_init
 *
 * @brief   Initialize simple peripheral profile, BLE related parameters.
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void simple_peripheral_init(void)
{
    // set local device name
	uint8_t local_name[] = "BLE TEST";
	gap_set_dev_name(local_name, sizeof(local_name));

	// Initialize security related settings.
	gap_security_param_t param =
	{
	    .mitm = false,
	    .ble_secure_conn = false,
	    .io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
	    .pair_init_mode = GAP_PAIRING_MODE_WAIT_FOR_REQ,
	    .bond_auth = false,
	    .password = 0,
	};
	gap_security_param_init(&param);
	gap_bond_manager_init(0x7D000, 0x7E000, 8, true);
  gap_bond_manager_delete_all();
	gap_set_cb_func(app_gap_evt_cb);
/********************************************************************/

	mac_addr_t addr;
	gap_address_get(&addr);
	co_printf("Local BDADDR: 0x%2X%2X%2X%2X%2X%2X\r\n", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);

	// Adding services to database
    sp_gatt_add_service();
	//speaker_gatt_add_service();				    //创建Speaker profile，
    


  gpio_set_dir(GPIO_PORT_A, GPIO_BIT_7, GPIO_DIR_IN);
 
	demo_LCD_APP();							            //显示屏
	//demo_CAPB18_APP();						            //气压计
//	demo_SHT3x_APP();						            //温湿度
	//gyro_dev_init();						            //加速度传感器
	
	//OS Timer
	
	 os_timer_init(&timer_refresh,timer_refresh_fun,NULL);//创建一个周期性1s定时的系统定时器
	 os_timer_start(&timer_refresh,400,1);
	/**
	 * @brief timer0 used for yaoshi
	 * 
	 */
#if 0 
   	 timer_init(TIMER0, 100, TIMER_FREE_RUN);
	 NVIC_EnableIRQ(TIMER0_IRQn);
	 timer_run(TIMER0);
#endif

/**
 * @brief  用于串口监控 1s监控一次  UartRxMonitor(1);
 * 
 */
	 timer_init(TIMER1, 1000, TIMER_FREE_RUN);
	 NVIC_EnableIRQ(TIMER1_IRQn);
	 timer_run(TIMER1);
	 

}


