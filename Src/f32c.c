/**
  ******************************************************************************
  * @file    f32c.c
  * @brief   F32C brushless motor driver protocol implementation
  *          USART1 (PA9 TX / PA10 RX) -> F32C, 115200 8N1
  ******************************************************************************
  */
#include "f32c.h"
#include "usart.h"
#include <string.h>

#define F32C_FRAME_HEAD  0x7A
#define F32C_FRAME_TAIL  0x7B

#define F32C_CMD_ENABLE     0x06
#define F32C_CMD_DISABLE   0x05
#define F32C_CMD_MODE      0x00
#define F32C_CMD_SPEED     0x01
#define F32C_CMD_POSITION  0x02
#define F32C_CMD_ACCEL     0x07
#define F32C_CMD_FEEDBACK  0x0E

#define F32C_TX_TIMEOUT_MS 50U
#define F32C_INTER_CMD_MS  2U   /* >=1ms required by manual */

static F32C_State_t g_f32c_state = {0};

/* External UART handle configured in usart.c (added in USER CODE section of main) */
extern UART_HandleTypeDef huart1;

/* Calc BCC: XOR of all bytes from 0x7A head to last DATA byte (official protocol:
 * BCC includes the frame head, verified vs manual: 7A^01^06=7D, 7A^02^06=7E) */
uint8_t F32C_CalcBCC(const uint8_t *frame, uint8_t len)
{
    uint8_t bcc = 0;
    uint8_t i;
    for (i = 0; i < len; i++) {
        bcc ^= frame[i];
    }
    return bcc;
}

void F32C_Init(void)
{
    memset(&g_f32c_state, 0, sizeof(g_f32c_state));
}

HAL_StatusTypeDef F32C_SendRaw(const uint8_t *frame, uint16_t len)
{
    HAL_StatusTypeDef st;
    st = HAL_UART_Transmit(&huart1, (uint8_t*)frame, len, F32C_TX_TIMEOUT_MS);
    HAL_Delay(F32C_INTER_CMD_MS);
    return st;
}

HAL_StatusTypeDef F32C_Enable(F32C_Axis_t axis)
{
    /* Frame: 7A ID 06 BCC 7B */
    uint8_t frame[5] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_ENABLE, 0, F32C_FRAME_TAIL};
    frame[3] = F32C_CalcBCC(&frame[0], 3);
    if (axis == F32C_AXIS_X) g_f32c_state.enabled |= 0x01;
    else                      g_f32c_state.enabled |= 0x02;
    return F32C_SendRaw(frame, 5);
}

HAL_StatusTypeDef F32C_Disable(F32C_Axis_t axis)
{
    uint8_t frame[5] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_DISABLE, 0, F32C_FRAME_TAIL};
    frame[3] = F32C_CalcBCC(&frame[0], 3);
    if (axis == F32C_AXIS_X) g_f32c_state.enabled &= ~0x01;
    else                      g_f32c_state.enabled &= ~0x02;
    return F32C_SendRaw(frame, 5);
}

HAL_StatusTypeDef F32C_SetMode(F32C_Axis_t axis, F32C_Mode_t mode)
{
    /* Frame: 7A ID 00 DATA_H DATA_L BCC 7B */
    uint8_t frame[7] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_MODE,
                        (uint8_t)(mode >> 8), (uint8_t)(mode & 0xFF), 0, F32C_FRAME_TAIL};
    frame[5] = F32C_CalcBCC(&frame[0], 5);
    return F32C_SendRaw(frame, 7);
}

HAL_StatusTypeDef F32C_SetSpeed(F32C_Axis_t axis, int16_t rpm)
{
    /* Frame: 7A ID 01 DATA_H DATA_L BCC 7B, 16-bit signed (two's complement) */
    uint16_t v = (uint16_t)rpm;
    uint8_t frame[7] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_SPEED,
                        (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), 0, F32C_FRAME_TAIL};
    frame[5] = F32C_CalcBCC(&frame[0], 5);
    return F32C_SendRaw(frame, 7);
}

HAL_StatusTypeDef F32C_SetPosition(F32C_Axis_t axis, float angle_deg)
{
    /* Multi-turn absolute position, DATA = angle * 10, 32-bit signed */
    int32_t v = (int32_t)(angle_deg * 10.0f);
    uint32_t uv = (uint32_t)v;
    uint8_t frame[9] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_POSITION,
                        (uint8_t)(uv >> 24), (uint8_t)(uv >> 16),
                        (uint8_t)(uv >> 8),  (uint8_t)(uv & 0xFF),
                        0, F32C_FRAME_TAIL};
    frame[7] = F32C_CalcBCC(&frame[0], 7);
    return F32C_SendRaw(frame, 9);
}

HAL_StatusTypeDef F32C_SetAccel(F32C_Axis_t axis, uint16_t accel)
{
    uint8_t frame[7] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_ACCEL,
                        (uint8_t)(accel >> 8), (uint8_t)(accel & 0xFF), 0, F32C_FRAME_TAIL};
    frame[5] = F32C_CalcBCC(&frame[0], 5);
    return F32C_SendRaw(frame, 7);
}

HAL_StatusTypeDef F32C_RequestFeedback(F32C_Axis_t axis, F32C_FbType_t type)
{
    /* Frame: 7A ID 0E TYPE BCC 7B */
    uint8_t frame[6] = {F32C_FRAME_HEAD, (uint8_t)axis, F32C_CMD_FEEDBACK,
                        (uint8_t)type, 0, F32C_FRAME_TAIL};
    frame[4] = F32C_CalcBCC(&frame[0], 4);
    return F32C_SendRaw(frame, 6);
}

/* Parse feedback frame: 7A ID TYPE HI16(2B) LO16(2B) BCC 7B (9 bytes) */
void F32C_RxParse(uint8_t *buf, uint16_t len)
{
    if (len != 9) return;
    if (buf[0] != F32C_FRAME_HEAD || buf[8] != F32C_FRAME_TAIL) return;
    uint8_t bcc = F32C_CalcBCC(&buf[0], 7);
    if (bcc != buf[7]) return;

    uint8_t id = buf[1];
    uint8_t type = buf[2];
    uint32_t v = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                 ((uint32_t)buf[5] << 8)  | (uint32_t)buf[6];
    int32_t sv = (int32_t)v;

    g_f32c_state.last_rx_tick = HAL_GetTick();

    if (id == F32C_AXIS_X) {
        switch (type) {
            case F32C_FB_SPEED:       g_f32c_state.speed_x = (int16_t)sv; break;
            case F32C_FB_TOTAL_ANGLE: g_f32c_state.total_angle_x = sv; break;
        }
    } else if (id == F32C_AXIS_Y) {
        switch (type) {
            case F32C_FB_SPEED:       g_f32c_state.speed_y = (int16_t)sv; break;
            case F32C_FB_TOTAL_ANGLE: g_f32c_state.total_angle_y = sv; break;
        }
    }
    if (type == F32C_FB_VOLTAGE) {
        g_f32c_state.bus_voltage = (uint16_t)v;
    }
}

const F32C_State_t* F32C_GetState(void) { return &g_f32c_state; }

void F32C_TestFrame(void)
{
    /* Send X-axis enable frame: 7A 01 06 07 7B (X轴ID=1) */
    (void)F32C_Enable(F32C_AXIS_X);
}
