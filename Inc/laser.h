/**
  ******************************************************************************
  * @file    laser.h
  * @brief   Laser on/off control (PB6 push-pull output)
  ******************************************************************************
  */
#ifndef __LASER_H
#define __LASER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_Toggle(void);
uint8_t Laser_GetState(void);

#ifdef __cplusplus
}
#endif
#endif /* __LASER_H */
