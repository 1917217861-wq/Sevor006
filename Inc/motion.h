/**
  ******************************************************************************
  * @file    motion.h
  * @brief   Motion control layer (open-loop trajectory + visual correction)
  ******************************************************************************
  */
#ifndef __MOTION_H
#define __MOTION_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x_angle_origin;   /* X axis angle at screen origin, deg */
    float y_angle_origin;   /* Y axis angle at screen origin, deg */
    /* screen four corners angle calibration (to be filled after measurement) */
    float x_angle_top_left;
    float x_angle_top_right;
    float x_angle_bottom_left;
    float x_angle_bottom_right;
    float y_angle_top_left;
    float y_angle_top_right;
    float y_angle_bottom_left;
    float y_angle_bottom_right;
    float tape_rotate_angle;  /* A4 tape rotation angle, deg */
} Screen_Calib_t;

void              Motion_Init(void);
void              Motion_CalibLoad(const Screen_Calib_t *calib);
HAL_StatusTypeDef Motion_Reset(void);                                  /* Cycle 3 */
HAL_StatusTypeDef Motion_FollowScreenEdge(uint32_t timeout_ms);       /* Cycle 4 */
HAL_StatusTypeDef Motion_FollowTape(float rotate_deg, uint32_t timeout_ms); /* Cycle 5 */
void              Motion_VisualCorrect(float dx_cm, float dy_cm);

#ifdef __cplusplus
}
#endif
#endif /* __MOTION_H */
