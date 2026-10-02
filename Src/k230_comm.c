/**
  ******************************************************************************
  * @file    k230_comm.c
  * @brief   K230 communication implementation (方案A)
  *          STM32 -> K230: 5-byte frame 0xBB + type + data + BCC + 0x55
  ******************************************************************************
  */
#include "k230_comm.h"
#include "usart.h"
#include <string.h>

/* RX frame (K230 -> STM32), reserved for Cycle 2+ */
#define K230_FRAME_HEADER 0xAA
#define K230_FRAME_TAIL   0x55
#define K230_FRAME_LEN    7

/* TX frame (STM32 -> K230) */
#define K230_TX_HEADER    0xBB
#define K230_TX_TAIL      0x55
#define K230_TX_LEN       5
#define K230_TX_TIMEOUT   100U

extern UART_HandleTypeDef huart2;

static K230_State_t g_k230_state = {0};

/* RX state machine (Cycle 2+, kept for future K230 feedback) */
typedef enum {
    K230_WAIT_HEADER,
    K230_RX_X_HIGH,
    K230_RX_X_LOW,
    K230_RX_Y_HIGH,
    K230_RX_Y_LOW,
    K230_RX_FLAG,
    K230_RX_TAIL
} K230_FrameState_t;

static K230_FrameState_t g_state = K230_WAIT_HEADER;
static uint8_t g_buf[K230_FRAME_LEN];
static uint8_t g_idx = 0;

void K230_Init(void)
{
    memset(&g_k230_state, 0, sizeof(g_k230_state));
    g_state = K230_WAIT_HEADER;
    g_idx = 0;
}

/* Build and send a 5-byte command frame: 0xBB + type + data + BCC + 0x55
 * BCC = type ^ data */
static HAL_StatusTypeDef K230_SendCmd(uint8_t type, uint8_t data)
{
    uint8_t frame[K230_TX_LEN];
    frame[0] = K230_TX_HEADER;
    frame[1] = type;
    frame[2] = data;
    frame[3] = (uint8_t)(type ^ data);   /* BCC */
    frame[4] = K230_TX_TAIL;
    return HAL_UART_Transmit(&huart2, frame, K230_TX_LEN, K230_TX_TIMEOUT);
}

HAL_StatusTypeDef K230_SendMode(WorkMode_t mode)
{
    /* MODE_PAUSE is handled by K230_SendPause, not here */
    if ((uint8_t)mode > (uint8_t)MODE_TAPE_ROTATE) {
        return HAL_ERROR;
    }
    return K230_SendCmd(K230_CMD_MODE, (uint8_t)mode);
}

HAL_StatusTypeDef K230_SendPause(uint8_t pause)
{
    return K230_SendCmd(K230_CMD_PAUSE, pause ? 1U : 0U);
}

HAL_StatusTypeDef K230_SendLaserState(uint8_t on)
{
    return K230_SendCmd(K230_CMD_LASER, on ? 1U : 0U);
}

void K230_OnRxByte(uint8_t data)
{
    /* Cycle 2+: parse K230 feedback frames.
     * Frame format: 0xAA + type + data(n) + BCC + 0x55
     * For now, minimal 7-byte legacy parser kept for debug. */
    g_buf[g_idx] = data;
    switch (g_state) {
        case K230_WAIT_HEADER:
            if (data == K230_FRAME_HEADER) {
                g_idx = 0;
                g_buf[g_idx] = data;
                g_idx = 1;
                g_state = K230_RX_X_HIGH;
            }
            return;
        case K230_RX_X_HIGH:
            g_idx = 2; g_state = K230_RX_X_LOW; return;
        case K230_RX_X_LOW:
            g_idx = 3; g_state = K230_RX_Y_HIGH; return;
        case K230_RX_Y_HIGH:
            g_idx = 4; g_state = K230_RX_Y_LOW; return;
        case K230_RX_Y_LOW:
            g_idx = 5; g_state = K230_RX_FLAG; return;
        case K230_RX_FLAG:
            g_idx = 6; g_state = K230_RX_TAIL; return;
        case K230_RX_TAIL:
            if (data == K230_FRAME_TAIL) {
                uint16_t x = ((uint16_t)g_buf[1] << 8) | g_buf[2];
                uint16_t y = ((uint16_t)g_buf[3] << 8) | g_buf[4];
                g_k230_state.laser_x_cm = (float)x;
                g_k230_state.laser_y_cm = (float)y;
                g_k230_state.detected = g_buf[5];
                g_k230_state.frame_count++;
                g_k230_state.last_rx_tick = HAL_GetTick();
            } else {
                g_k230_state.err_count++;
            }
            g_state = K230_WAIT_HEADER;
            g_idx = 0;
            return;
        default:
            g_state = K230_WAIT_HEADER;
            g_idx = 0;
            return;
    }
}

const K230_State_t* K230_GetState(void) { return &g_k230_state; }

void K230_TestSend(void)
{
    /* Debug: send a MODE_RESET frame BB 01 01 00 55 */
    (void)K230_SendMode(MODE_RESET);
}
