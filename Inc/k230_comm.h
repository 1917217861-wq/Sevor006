/**
  ******************************************************************************
  * @file    k230_comm.h
  * @brief   K230 communication (USART2 PA2/PA3) - 方案A
  *          STM32 -> K230: 0xBB + type + data + BCC + 0x55 (5 bytes)
  *            type=0x01 mode  / type=0x02 pause / type=0x03 laser
  *          K230  -> STM32: 0xAA + type + data(n) + BCC + 0x55 (Cycle 2+)
  ******************************************************************************
  */
#ifndef __K230_COMM_H
#define __K230_COMM_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reuse WorkMode_t from main.h */

/* K230 feedback state (Cycle 2+, reserved for future use) */
typedef struct {
    float    laser_x_cm;     /* laser point X on screen, cm, origin at screen center */
    float    laser_y_cm;     /* laser point Y on screen, cm */
    uint8_t  detected;       /* 1 = laser detected, 0 = not detected */
    uint32_t frame_count;    /* accumulated valid frame count */
    uint32_t err_count;      /* bad frame count */
    uint32_t last_rx_tick;
} K230_State_t;

/* TX frame type codes (STM32 -> K230) */
#define K230_CMD_MODE    0x01U   /* switch work mode */
#define K230_CMD_PAUSE   0x02U   /* pause/resume */
#define K230_CMD_LASER   0x03U   /* laser on/off */

void              K230_Init(void);
HAL_StatusTypeDef K230_SendMode(WorkMode_t mode);
HAL_StatusTypeDef K230_SendPause(uint8_t pause);
HAL_StatusTypeDef K230_SendLaserState(uint8_t on);
void              K230_OnRxByte(uint8_t data);   /* call from USART2 RxCplt callback */
const K230_State_t* K230_GetState(void);
void              K230_TestSend(void);            /* debug */

#ifdef __cplusplus
}
#endif
#endif /* __K230_COMM_H */
