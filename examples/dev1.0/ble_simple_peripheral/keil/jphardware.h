#ifndef _JP_HARDWARE_H_
#define _JP_HARDWARE_H_

#include "driver_iomux.h"
#include "driver_gpio.h"
enum jp_port
{
    GPIO_A,
    GPIO_B,
    GPIO_C,
    GPIO_D,
};

void GPIO_SetBits(uint8_t PORT, uint8_t PIN);
void GPIO_ResetBits(uint8_t PORT, uint8_t PIN);


#endif
