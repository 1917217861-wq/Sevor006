/**
  ******************************************************************************
  * @file    key.h
  * @brief   Key scan with debounce (PB0 mode / PB1 reset / PB5 pause)
  ******************************************************************************
  */
#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    KEY_NONE = 0,
    KEY_MODE,
    KEY_RESET,
    KEY_PAUSE
} KeyEvent_t;

void        Key_Init(void);
KeyEvent_t  Key_Scan(void);

#ifdef __cplusplus
}
#endif
#endif /* __KEY_H */
