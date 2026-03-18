/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Single DC motor PID speed control with encoder
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX              255
#define PWM_MIN             -255

#define CONTROL_PERIOD_MS    20U          // PID 제어 주기 20ms
#define ENCODER_CPR          3172.0f      // 1회전당 카운트 수
#define MOTOR_DEADZONE       20           // 정지마찰 보상용 최소 PWM

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* 엔코더 카운트 */
volatile long long totalEncoderCount = 0;

/* PID 변수 */
float target_rps   = 1.0f;   // 목표 속도 [rev/s], 테스트용 기본값
float current_rps  = 0.0f;

float Kp = 4.0f;
float Ki = 8.0f;
float Kd = 1.6f;

float error       = 0.0f;
float prev_error  = 0.0f;
float integral    = 0.0f;
float derivative  = 0.0f;
float pid_output  = 0.0f;

int motor_pwm = 0;

/* 제어 주기용 */
uint32_t last_control_time = 0;

/* 디버그 출력용 */
char tx_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  if (ch == '\n')
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void Motor_SetDirection(int dir);
void Motor_SetPWM(int pwm);
void Motor_Stop(void);
void Motor_PID_Control(void);
void check_pwm_limit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  모터 방향 설정
  * @param  dir: 1이면 정회전, -1이면 역회전
  */
void Motor_SetDirection(int dir)
{
  if (dir >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  PWM 출력
  * @param  pwm: -255 ~ 255
  */
void Motor_SetPWM(int pwm)
{
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  if (pwm < PWM_MIN) pwm = PWM_MIN;

  if (pwm == 0)
  {
    Motor_Stop();
    return;
  }

  if (pwm > 0)
  {
    Motor_SetDirection(1);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
  }
  else
  {
    Motor_SetDirection(-1);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, -pwm);
  }
}

/**
  * @brief  모터 정지
  */
void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  PWM 범위 제한
  */
void check_pwm_limit(void)
{
  if (motor_pwm > PWM_MAX) motor_pwm = PWM_MAX;
  if (motor_pwm < PWM_MIN) motor_pwm = PWM_MIN;
}

/**
  * @brief  PID 속도 제어
  * @note   CONTROL_PERIOD_MS마다 실행
  */
void Motor_PID_Control(void)
{
  static long long prevEncoderCount = 0;
  uint32_t now = HAL_GetTick();

  if ((now - last_control_time) >= CONTROL_PERIOD_MS)
  {
    float dt_sec;
    long long deltaCount;

    dt_sec = (now - last_control_time) / 1000.0f;
    last_control_time = now;

    /* 현재 속도 계산 */
    deltaCount = totalEncoderCount - prevEncoderCount;
    prevEncoderCount = totalEncoderCount;

    current_rps = ((float)deltaCount / ENCODER_CPR) / dt_sec;

    /* PID 계산 */
    error = target_rps - current_rps;
    integral += error * dt_sec;
    derivative = (error - prev_error) / dt_sec;

    pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    /* 증분형처럼 PWM 누적 */
    motor_pwm += (int)pid_output;

    /* 목표 속도 0이면 완전 정지 */
    if (fabsf(target_rps) < 0.001f)
    {
      motor_pwm = 0;
      integral = 0.0f;
      prev_error = 0.0f;
    }
    else
    {
      /* 정지마찰 보상 */
      if (motor_pwm > 0 && motor_pwm < MOTOR_DEADZONE)
        motor_pwm = MOTOR_DEADZONE;
      else if (motor_pwm < 0 && motor_pwm > -MOTOR_DEADZONE)
        motor_pwm = -MOTOR_DEADZONE;
    }

    check_pwm_limit();
    Motor_SetPWM(motor_pwm);

    prev_error = error;

    /* 디버깅 출력 */
    printf("target=%.3f rps, current=%.3f rps, err=%.3f, pwm=%d\n",
           target_rps, current_rps, error, motor_pwm);
  }
}

/**
  * @brief  엔코더 4체배 카운팅
  * @note   Left_pulse_A / Left_pulse_B 사용
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Left_pulse_A_Pin)
  {
    if (HAL_GPIO_ReadPin(GPIOB, Left_pulse_A_Pin) ==
        HAL_GPIO_ReadPin(GPIOB, Left_pulse_B_Pin))
    {
      totalEncoderCount--;
    }
    else
    {
      totalEncoderCount++;
    }
  }
  else if (GPIO_Pin == Left_pulse_B_Pin)
  {
    if (HAL_GPIO_ReadPin(GPIOB, Left_pulse_A_Pin) ==
        HAL_GPIO_ReadPin(GPIOB, Left_pulse_B_Pin))
    {
      totalEncoderCount++;
    }
    else
    {
      totalEncoderCount--;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  Motor_Stop();
  last_control_time = HAL_GetTick();

  /*
   * 테스트용 목표 속도 설정
   * 예:
   *  1.0f  -> 정회전 1 rev/s
   *  0.5f  -> 정회전 0.5 rev/s
   * -1.0f  -> 역회전 1 rev/s
   *  0.0f  -> 정지
   */
  target_rps = 1.0f;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    Motor_PID_Control();
    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 255;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 출력 초기 상태 */
  HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin, GPIO_PIN_RESET);

  /* Left_IN2_Pin */
  GPIO_InitStruct.Pin = Left_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Left_IN1_Pin */
  GPIO_InitStruct.Pin = Left_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 엔코더 입력 */
  GPIO_InitStruct.Pin = Left_pulse_A_Pin | Left_pulse_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
