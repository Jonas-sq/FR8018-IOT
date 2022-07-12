#include "jphardware.h"

void GPIO_SetBits(uint8_t PORT, uint8_t PIN)
{
    switch(PORT)
    {
        case GPIO_A:
        {
            gpio_porta_write(gpio_porta_read() | (1<<PIN) );
            break;
        }
        case GPIO_B:
        {
            gpio_portb_write(gpio_portc_read() | (1<<PIN) );
            break;
        }
        case GPIO_C:
        {
            gpio_portc_write(gpio_portc_read() | (1<<PIN) );
            break;
        }
        case GPIO_D:
        {
            gpio_portd_write(gpio_portd_read() | (1<<PIN) );
            break;
        }
    }
    
}



