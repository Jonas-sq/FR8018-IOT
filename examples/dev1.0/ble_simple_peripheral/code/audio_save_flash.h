#ifndef __AUDIO_SAVE_FLASH_H_
#define __AUDIO_SAVE_FLASH_H_

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
#endif


