#ifndef _SPEAKER_H
#define _SPEAKER_H

typedef enum {
    UART_REV_STATE_FOUND_NULL,
    UART_REV_STATE_FOUND_HEAD,
    UART_REV_STATE_FOUND_CMD,
    UART_REV_STATE_FOUND_LEN_H,
    UART_REV_STATE_FOUND_LEN_L,
    UART_REV_STATE_FOUND_DATA,
    UART_REV_STATE_UNKOWN,
} uart_rev_state_type_t;


void audio_uart_init(void);


#define PA_ENABLE      gpio_portb_write(gpio_portb_read() | (1<<GPIO_BIT_4) )
#endif
