/**
  ******************************************************************************
  * @file    f32c.h
  * @brief   F32C brushless motor driver protocol layer
  *          WHEELTEC F32C 2-axis gimbal (X axis ID=0, Y axis ID=1)
  *          Protocol: 7A + ID + CMD + DATA + BCC + 7B
  ******************************************************************************
  */
#ifndef __F32C_H
#define __F32C_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* F32C motor axis (also equals motor ID) */
typedef enum {
    F32C_AXIS_X = 1,    /* X axis motor, ID = 1 */
    F32C_AXIS_Y = 2     /* Y axis motor, ID = 2 */
} F32C_Axis_t;

/* F32C control mode (mode select command data 0x0000~0x0004) */
typedef enum {
    F32C_MODE_SPEED             = 0x0000,
    F32C_MODE_MULTI_TURN_T      = 0x0001,  /* multi-turn position, T trajectory (default) */
    F32C_MODE_SINGLE_TURN_T     = 0x0002,
    F32C_MODE_MULTI_TURN_DIRECT = 0x0003,
    F32C_MODE_SINGLE_TURN_DIRECT = 0x0004
} F32C_Mode_t;

/* F32C feedback type */
typedef enum {
    F32C_FB_SPEED       = 0x00,  /* speed RPM */
    F32C_FB_TOTAL_ANGLE = 0x01,  /* total turned angle, unit 0.1 deg */
    F32C_FB_MECH_ANGLE  = 0x02,  /* mechanical angle, unit 0.1 deg */
    F32C_FB_ACCEL       = 0x03,
    F32C_FB_VOLTAGE     = 0x04   /* bus voltage, unit 0.01 V */
} F32C_FbType_t;

/* F32C state snapshot */
typedef struct {
    int32_t  total_angle_x;   /* X total angle, unit 0.1 deg */
    int32_t  total_angle_y;   /* Y total angle, unit 0.1 deg */
    int16_t  speed_x;         /* X speed RPM */
    int16_t  speed_y;
    uint16_t bus_voltage;      /* bus voltage, unit 0.01 V */
    uint32_t last_rx_tick;     /* last feedback tick */
    uint8_t  enabled;          /* motors enabled flag */
} F32C_State_t;

/* Public API */
void              F32C_Init(void);
uint8_t           F32C_CalcBCC(const uint8_t *frame, uint8_t len);  /* len: bytes between head and tail */
HAL_StatusTypeDef F32C_SendRaw(const uint8_t *frame, uint16_t len);
HAL_StatusTypeDef F32C_Enable(F32C_Axis_t axis);
HAL_StatusTypeDef F32C_Disable(F32C_Axis_t axis);
HAL_StatusTypeDef F32C_SetMode(F32C_Axis_t axis, F32C_Mode_t mode);
HAL_StatusTypeDef F32C_SetSpeed(F32C_Axis_t axis, int16_t rpm);
HAL_StatusTypeDef F32C_SetPosition(F32C_Axis_t axis, float angle_deg);
HAL_StatusTypeDef F32C_SetAccel(F32C_Axis_t axis, uint16_t accel);
HAL_StatusTypeDef F32C_RequestFeedback(F32C_Axis_t axis, F32C_FbType_t type);

void              F32C_RxParse(uint8_t *buf, uint16_t len);
const F32C_State_t* F32C_GetState(void);

void              F32C_TestFrame(void);   /* debug: send enable frame to verify UART */

#ifdef __cplusplus
}
#endif
#endif /* __F32C_H */
