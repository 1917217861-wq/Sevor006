/**
  ******************************************************************************
  * @file    laser.c
  * @brief   Laser GPIO control (PB6)
  ******************************************************************************
  */
#include "laser.h"
#include "gpio.h"

#define LASER_PORT      GPIOB
#define LASER_PIN       GPIO_PIN_6
#define LED_PORT         GPIOB
#define LED_PIN          GPIO_PIN_7

static uint8_t g_laser_state = 0;

void Laser_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LASER_PIN | LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    g_laser_state = 0;
}

void Laser_On(void)
{
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    g_laser_state = 1;
}

void Laser_Off(void)
{
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    g_laser_state = 0;
}

void Laser_Toggle(void)
{
    if (g_laser_state) Laser_Off();
    else               Laser_On();
}

uint8_t Laser_GetState(void) { return g_laser_state; }
