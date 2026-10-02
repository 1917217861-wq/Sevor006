/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 2023 NUEDC E problem - motion target control system
  *                  Hardware (方案A): STM32F103C8Tx + WHEELTEC F32C gimbal + K230
  *                  K230 直连 F32C 控电机; STM32 只做人机交互
  *                  USART2 (PA2/PA3) -> K230 (发模式/暂停/激光指令)
  *                  PB0/PB1/PB5 keys, PB6 laser, PB7 LED
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "k230_comm.h"
#include "laser.h"
#include "key.h"
#include "debug.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAIN_LOOP_PERIOD_MS  10U   /* main loop period */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

WorkMode_t g_current_mode = MODE_IDLE;
WorkMode_t g_mode_before_pause = MODE_IDLE;   /* saved mode for resume */
uint8_t    g_pause_flag = 0;
uint8_t    g_laser_state = 0;
uint8_t    g_k230_rx_byte = 0;                /* single byte rx buffer for K230 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void Mode_Enter(WorkMode_t mode);
static void Mode_Step(WorkMode_t mode);
static void Laser_SetState(uint8_t on);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Sync laser GPIO + state + notify K230 */
static void Laser_SetState(uint8_t on)
{
    if (on) Laser_On();
    else    Laser_Off();
    g_laser_state = on;
    (void)K230_SendLaserState(on);
}

/* Enter a new mode (called on mode change) */
static void Mode_Enter(WorkMode_t mode)
{
    switch (mode) {
        case MODE_IDLE:
            DBG_LOG("MODE: IDLE");
            (void)K230_SendMode(MODE_IDLE);
            break;
        case MODE_RESET:
            DBG_LOG("MODE: RESET");
            (void)K230_SendMode(MODE_RESET);
            break;
        case MODE_EDGE:
            DBG_LOG("MODE: EDGE");
            (void)K230_SendMode(MODE_EDGE);
            break;
        case MODE_TAPE:
            DBG_LOG("MODE: TAPE");
            (void)K230_SendMode(MODE_TAPE);
            break;
        case MODE_TAPE_ROTATE:
            DBG_LOG("MODE: TAPE_ROTATE");
            (void)K230_SendMode(MODE_TAPE_ROTATE);
            break;
        case MODE_PAUSE:
            DBG_LOG("MODE: PAUSE");
            (void)K230_SendPause(1);
            /* Laser stays in current state during pause (赛题要求制动便于测量) */
            break;
        default:
            break;
    }
}

/* Per-loop action for current mode (STM32只管按键+发指令,轨迹由K230执行) */
static void Mode_Step(WorkMode_t mode)
{
    switch (mode) {
        case MODE_RESET:
        case MODE_EDGE:
        case MODE_TAPE:
        case MODE_TAPE_ROTATE:
            /* K230 端执行轨迹; STM32 无需干预 */
            break;
        case MODE_IDLE:
        case MODE_PAUSE:
        default:
            break;
    }
}

/**
  * @brief  K230 receive callback (called per byte from HAL).
  *         USART2 is shared with printf debug; K230 feedback frames parsed here.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        K230_OnRxByte(g_k230_rx_byte);
        HAL_UART_Receive_IT(&huart2, &g_k230_rx_byte, 1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 方案A: 不需要 USART1 (原方案B用于接F32C,现已废弃) */
  /* MX_USART1_UART_Init() 已移除 */

  /* Module init (方案A: 只剩 K230通信 + 激光 + 按键 + 调试) */
  K230_Init();
  Laser_Init();
  Key_Init();
  Debug_Init();

  /* Laser off at boot */
  Laser_SetState(0);

  /* Start K230 byte-by-byte receive (USART2) */
  HAL_UART_Receive_IT(&huart2, &g_k230_rx_byte, 1);

  DBG_LOG("BOOT OK (方案A: K230直连F32C)");
  DBG_LOG("MODE: IDLE");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Key scan: mode / reset / pause */
    KeyEvent_t key = Key_Scan();
    if (key != KEY_NONE) {
        switch (key) {
            case KEY_MODE:
                if (g_pause_flag) {
                    /* ignore mode switch while paused */
                    DBG_LOG("KEY_MODE (ignored, paused)");
                } else {
                    /* cycle: IDLE -> RESET -> EDGE -> TAPE -> TAPE_ROTATE -> IDLE */
                    WorkMode_t next = (WorkMode_t)((g_current_mode + 1) % MODE_PAUSE);
                    g_current_mode = next;
                    Mode_Enter(g_current_mode);
                }
                break;
            case KEY_RESET:
                g_current_mode = MODE_RESET;
                g_pause_flag = 0;
                Mode_Enter(MODE_RESET);   /* 自动开激光 */
                break;
            case KEY_PAUSE:
                if (g_pause_flag) {
                    /* resume */
                    g_pause_flag = 0;
                    g_current_mode = g_mode_before_pause;
                    /* 🔧 恢复必须显式发 pause=0, 不能靠 Mode_Enter 发模式帧:
                     *    K230 端对模式帧做了去重(mode没变就不回调), 暂停期间模式值没变,
                     *    所以恢复时发的 mode 会被 K230 直接丢掉 -> on_pause(0) 永不触发 ->
                     *    K230 的 pause_flag 卡在1 -> **之后每次按暂停都被它自己去重掉**,
                     *    现象就是"按暂停键, 激光还是继续走"。
                     *    (2026-09-30 实测) K230 收到 pause=0 会走 trajectory.resume()
                     *    -> start(paused_mode), 不需要STM32再发模式帧 */
                    (void)K230_SendPause(0);
                    DBG_LOG("RESUME");
                } else {
                    g_pause_flag = 1;
                    g_mode_before_pause = g_current_mode;
                    g_current_mode = MODE_PAUSE;
                    Mode_Enter(MODE_PAUSE);
                }
                break;
            default:
                break;
        }
    }

    /* 2. Mode step (state machine tick) */
    Mode_Step(g_current_mode);

    /* 3. Periodic heartbeat: send current mode to K230 every ~500ms */
    {
        static uint32_t last_send_tick = 0;
        if (HAL_GetTick() - last_send_tick > 500U) {
            last_send_tick = HAL_GetTick();
            if (g_current_mode != MODE_PAUSE) {
                (void)K230_SendMode(g_current_mode);
            }
        }
    }

    HAL_Delay(MAIN_LOOP_PERIOD_MS);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
      /* blink LED fast on error */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and the line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
