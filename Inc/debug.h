/**
  ******************************************************************************
  * @file    debug.h
  * @brief   Debug logging via USART2 printf redirect
  ******************************************************************************
  */
#ifndef __DEBUG_H
#define __DEBUG_H

#include "main.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void Debug_Init(void);

#define DBG_LOG(fmt, ...) do { \
    printf("[%lu] " fmt "\r\n", (unsigned long)HAL_GetTick(), ##__VA_ARGS__); \
} while (0)

#ifdef __cplusplus
}
#endif
#endif /* __DEBUG_H */
