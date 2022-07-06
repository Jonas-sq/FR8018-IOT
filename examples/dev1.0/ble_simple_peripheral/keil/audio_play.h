#ifndef _AUDIO_PLAY_H_
#define _AUDIO_PLAY_H_


#include "os_task.h"
#include "os_msg_q.h"

void PA_init_pins(void);
void audio_task_msg_post(uint8_t event_id);
void audio_task_init(void);
uint8_t audio_play_ready(uint32_t flash_addr);
#endif
