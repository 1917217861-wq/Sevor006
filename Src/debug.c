/**
  ******************************************************************************
  * @file    debug.c
  * @brief   printf redirect to USART2 (shared with K230, temporary for debug)
  ******************************************************************************
  */
#include "debug.h"
#include "usart.h"

extern UART_HandleTypeDef huart2;

#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    (void)f;
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}

void Debug_Init(void)
{
    /* Nothing extra; USART2 already initialized in MX_USART2_UART_Init */
}
