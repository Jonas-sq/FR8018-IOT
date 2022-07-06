/**
 * Copyright (c) 2019, Freqchip
 * 
 * All rights reserved.
 * 
 * 
 */

/*
 * INCLUDE FILES
 */
#include <stdio.h>
#include <string.h>

#include "gap_api.h"
#include "gatt_api.h"

#include "os_timer.h"
#include "os_mem.h"
#include "sys_utils.h"
#include "button.h"
#include "jump_table.h"
#include "user_task.h"
#include "driver_plf.h"
#include "driver_system.h"
#include "driver_i2s.h"
#include "driver_pmu.h"
#include "driver_uart.h"
#include "driver_rtc.h"
#include "ble_simple_peripheral.h"
#include "simple_gatt_service.h"

#include "speaker.h"
#include "audio_play.h"
#include "decoder.h"

#include "driver_flash.h"
#include "driver_efuse.h"
#include "flash_usage_config.h"
#include "driver_gpio.h"

#include "driver_codec.h"
////////////////////////////////////TIMER
#include <stdint.h>
#include "co_printf.h"
#include "driver_timer.h"

#include "tongxin.h"

const struct jump_table_version_t _jump_table_version __attribute__((section("jump_table_3"))) = 
{
    .firmware_version = 0x00000001,
};

const struct jump_table_image_t _jump_table_image __attribute__((section("jump_table_1"))) =
{
    .image_type = IMAGE_TYPE_APP,
    .image_size = 0x19000,      
};

__attribute__((section("ram_code"))) void pmu_gpio_isr_ram(void)
{
    uint32_t gpio_value = ool_read32(PMU_REG_GPIOA_V);
    
    button_toggle_detected(gpio_value);
    ool_write32(PMU_REG_PORTA_LAST, gpio_value);
}

/*********************************************************************
 * @fn      user_custom_parameters
 *
 * @brief   initialize several parameters, this function will be called 
 *          at the beginning of the program. 
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void user_custom_parameters(void)
{
	   struct chip_unique_id_t id_data;
	
    __jump_table.addr.addr[0] = 0xBe;
    __jump_table.addr.addr[1] = 0xAD;
    __jump_table.addr.addr[2] = 0xD0;
    __jump_table.addr.addr[3] = 0xF0;
    __jump_table.addr.addr[4] = 0x80;
    __jump_table.addr.addr[5] = 0x10;
	
	    id_data.unique_id[5] |= 0xc0; // random addr->static addr type:the top two bit must be 1.
    memcpy(__jump_table.addr.addr,id_data.unique_id,6);
    __jump_table.system_clk = SYSTEM_SYS_CLK_48M;
    jump_table_set_static_keys_store_offset(JUMP_TABLE_STATIC_KEY_OFFSET);
    
//    __jump_table.system_clk = SYSTEM_SYS_CLK_48M;
//	__jump_table.system_option &= ~(SYSTEM_OPTION_SLEEP_ENABLE);//取消sleep模式
//	jump_table_set_static_keys_store_offset(0x7d000);
}

/*********************************************************************
 * @fn      user_entry_before_sleep_imp
 *
 * @brief   Before system goes to sleep mode, user_entry_before_sleep_imp()
 *          will be called, MCU peripherals can be configured properly before 
 *          system goes to sleep, for example, some MCU peripherals need to be
 *          used during the system is in sleep mode. 
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
__attribute__((section("ram_code"))) void user_entry_before_sleep_imp(void)
{
}

/*********************************************************************
 * @fn      user_entry_after_sleep_imp
 *
 * @brief   After system wakes up from sleep mode, user_entry_after_sleep_imp()
 *          will be called, MCU peripherals need to be initialized again, 
 *          this can be done in user_entry_after_sleep_imp(). MCU peripherals
 *          status will not be kept during the sleep. 
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
__attribute__((section("ram_code"))) void user_entry_after_sleep_imp(void)
{
    /* set PA2 and PA3 for AT command interface */
    system_set_port_pull(GPIO_PA2, true);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_2, PORTA2_FUNC_UART1_RXD);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_3, PORTA3_FUNC_UART1_TXD);
    
    //system_sleep_disable();

//    if(__jump_table.system_option & SYSTEM_OPTION_ENABLE_HCI_MODE)
//    {
//        system_set_port_pull(GPIO_PA4, true);
//        system_set_port_mux(GPIO_PORT_A, GPIO_BIT_4, PORTA4_FUNC_UART0_RXD);
//        system_set_port_mux(GPIO_PORT_A, GPIO_BIT_5, PORTA5_FUNC_UART0_TXD);
//        uart_init(UART0, BAUD_RATE_115200);
//        NVIC_EnableIRQ(UART0_IRQn);

//        system_sleep_disable();
//    }

    uart_init(UART1, BAUD_RATE_115200);
    NVIC_EnableIRQ(UART1_IRQn);

    // Do some things here, can be uart print

    NVIC_EnableIRQ(PMU_IRQn);
}


/*********************************************************************
 * @fn      user_entry_before_ble_init
 *
 * @brief   Code to be executed before BLE stack to be initialized.
 *          Power mode configurations, PMU part driver interrupt enable, MCU 
 *          peripherals init, etc. 
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void user_entry_before_ble_init(void)
{    
    /* set system power supply in BUCK mode */
    pmu_set_sys_power_mode(PMU_SYS_POW_BUCK);

    pmu_enable_irq(PMU_ISR_BIT_ACOK
                   | PMU_ISR_BIT_ACOFF
                   | PMU_ISR_BIT_ONKEY_PO
                   | PMU_ISR_BIT_OTP
                   | PMU_ISR_BIT_LVD
                   | PMU_ISR_BIT_BAT
                   | PMU_ISR_BIT_ONKEY_HIGH);
    NVIC_EnableIRQ(PMU_IRQn);
    
    // Enable UART print.
    system_set_port_pull(GPIO_PA2, true);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_2, PORTA2_FUNC_UART1_RXD);
    system_set_port_mux(GPIO_PORT_A, GPIO_BIT_3, PORTA3_FUNC_UART1_TXD);
    NVIC_EnableIRQ(UART1_IRQn);
		uart_init(UART1, BAUD_RATE_115200);    
 
//    if(__jump_table.system_option & SYSTEM_OPTION_ENABLE_HCI_MODE)
//    {
//        /* use PC4 and PC5 for HCI interface */
//        system_set_port_pull(GPIO_PA4, true);
//        system_set_port_mux(GPIO_PORT_A, GPIO_BIT_4, PORTA4_FUNC_UART0_RXD);
//        system_set_port_mux(GPIO_PORT_A, GPIO_BIT_5, PORTA5_FUNC_UART0_TXD);
//    }

//    /* used for debug, reserve 3S for j-link once sleep is enabled. */
//    if(__jump_table.system_option & SYSTEM_OPTION_SLEEP_ENABLE)
//    {
//        co_delay_100us(10000);
//        co_delay_100us(10000);
//        co_delay_100us(10000);
//    }
}

/*********************************************************************
 * @fn      user_entry_after_ble_init
 *
 * @brief   Main entrancy of user application. This function is called after BLE stack
 *          is initialized, and all the application code will be executed from here.
 *          In that case, application layer initializtion can be startd here. 
 *
 * @param   None. 
 *       
 *
 * @return  None.
 */
void user_entry_after_ble_init(void)
{
    co_printf("BLE Peripheral\r\n");
    system_sleep_disable();
	
	  audio_uart_init();
	  user_lwrb_init();	
	  PA_init_pins();																													/*PB4设置为 上拉模式 输出模式*/
	  audio_task_init();	
		  	
		//tongxin_task_init();			
	
		system_set_port_pull(GPIO_PB3, true);																		/*PB3设置为 上拉模式*/
		system_set_port_mux(GPIO_PORT_B, GPIO_BIT_3, PORTB3_FUNC_B3);						/*PB3设置为 PORTB3_FUNC_B3 功能*/
		gpio_set_dir(GPIO_PORT_B, GPIO_BIT_3, GPIO_DIR_OUT);										/*PB3设置为 输出模式*/
	
		//按键初始化 PD3  （振动监测：低电平有效）
		pmu_set_pin_pull(GPIO_PORT_D, (1<<GPIO_BIT_3), true);										/*PD3设置为 低功耗上拉模式*/
		pmu_port_wakeup_func_set(GPIO_PD3);																			/*PD3       使能低功耗唤醒*/
		button_init(GPIO_PD3);																									/*PD3 			按键初始化*/
	///////////////////////////////////////
		//串口0 初始化 波特率9600
		system_set_port_pull(GPIO_PB2, true);																		/*PB2设置为 上拉模式*/
		system_set_port_mux(GPIO_PORT_B, GPIO_BIT_2, PORTB2_FUNC_UART0_RXD);		/*PB2设置为 PORTB2_FUNC_UART0_RXD */
		system_set_port_mux(GPIO_PORT_B, GPIO_BIT_1, PORTB3_FUNC_UART0_TXD);		/*PB2设置为 PORTB3_FUNC_UART0_TXD */
		NVIC_EnableIRQ(UART0_IRQn);
		uart_init(UART0, BAUD_RATE_9600);
		
		/*  485_TXRX_EN  */
		system_set_port_pull(GPIO_PB5, true);																		/*PB5设置为 上拉模式*/
		system_set_port_mux(GPIO_PORT_B, GPIO_BIT_5, PORTB5_FUNC_B5);						/*PB5设置为 PORTB5_FUNC_B5*/
		gpio_set_dir(GPIO_PORT_B, GPIO_BIT_5, GPIO_DIR_OUT);										/*PB5设置为 输出模式*/
		//gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_5) );//1	
	   gpio_portb_write(gpio_portb_read() 	& ~ (1<<GPIO_BIT_5) );//0
		 /*
				gpio_portb_write()        set gpio PORTB output value.
				gpio_portb_read()					get current value of gpio PORTB
		 */

//		 uint8_t test[5]={0x01,0x02,0x03,0x04,0x05};	
//		 gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_5) );//1	
//	  
//	   uart_write(UART0,test,5);
									
	
    // User task initialization, for buttons.
    user_task_init();
    
    // Application layer initialization, can included bond manager init, 
    // advertising parameters init, scanning parameter init, GATT service adding, etc.    
    simple_peripheral_init();
		
//gpio_portb_write(gpio_portb_read() 	& ~ (1<<GPIO_BIT_5) );//0
}
