#ifndef _TONGXIN_H_
#define _TONGXIN_H_


#include "os_task.h"
#include "os_msg_q.h"


void tongxin_task_msg_post(uint8_t event_id);
void tongxin_task_init(void);
extern uint8_t cmdArrived;
extern uint8_t BLE_cmd;
void timer_refresh_fun(void *arg);
//extern void UartRxMonitor(unsigned char ms); 

#endif
