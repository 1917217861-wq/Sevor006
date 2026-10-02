/**
  ******************************************************************************
  * @file    key.c
  * @brief   Key scan implementation (PB0/PB1/PB5, active low)
  ******************************************************************************
  */
#include "key.h"
#include "gpio.h"

#define KEY_MODE_PORT   GPIOB
#define KEY_MODE_PIN    GPIO_PIN_0
#define KEY_RESET_PORT  GPIOB
#define KEY_RESET_PIN   GPIO_PIN_1
#define KEY_PAUSE_PORT  GPIOB
#define KEY_PAUSE_PIN   GPIO_PIN_5

#define KEY_DEBOUNCE_MS 20U

static uint8_t Key_ReadRaw(void)
{
    if (HAL_GPIO_ReadPin(KEY_MODE_PORT, KEY_MODE_PIN) == GPIO_PIN_RESET)  return 1;
    if (HAL_GPIO_ReadPin(KEY_RESET_PORT, KEY_RESET_PIN) == GPIO_PIN_RESET) return 2;
    if (HAL_GPIO_ReadPin(KEY_PAUSE_PORT, KEY_PAUSE_PIN) == GPIO_PIN_RESET) return 3;
    return 0;
}

void Key_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = KEY_MODE_PIN | KEY_RESET_PIN | KEY_PAUSE_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

KeyEvent_t Key_Scan(void)
{
    uint8_t k = Key_ReadRaw();
    if (k == 0) return KEY_NONE;
    HAL_Delay(KEY_DEBOUNCE_MS);
    uint8_t k2 = Key_ReadRaw();
    if (k != k2) return KEY_NONE;
    while (Key_ReadRaw() != 0) { /* wait release */ }
    HAL_Delay(KEY_DEBOUNCE_MS);
    switch (k) {
        case 1: return KEY_MODE;
        case 2: return KEY_RESET;
        case 3: return KEY_PAUSE;
        default: return KEY_NONE;
    }
}
