/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Este programa implementa un control de acceso con teclado matricial y una
  * pantalla OLED SSD1306. La comunicación UART se ha sustituido por la
  * pantalla para mostrar el estado del sistema.
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_driver.h"
#include "ring_buffer.h"
#include "keypad_driver.h"
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SCREEN_STATE_ENTER_PIN,
    SCREEN_STATE_GRANTED,
    SCREEN_STATE_DENIED
} ScreenState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASSWORD "9A#*"
#define PASSWORD_LEN 4
#define DEBOUNCE_TIME_MS 200
#define FEEDBACK_LED_TIME_MS 100
#define SUCCESS_LED_TIME_MS 5000
#define MSG_DISPLAY_TIME_MS 2500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
led_handle_t led1 = { .port = GPIOA, .pin = GPIO_PIN_5 };

keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}
};

#define KEYPAD_BUFFER_LEN 16
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN];
ring_buffer_t keypad_rb;

char entered_password[PASSWORD_LEN + 1] = {0};
uint8_t password_index = 0;
ScreenState_t current_screen_state = SCREEN_STATE_ENTER_PIN;

uint32_t last_key_press_time = 0;
uint32_t led_timer_start = 0;
uint32_t led_on_duration = 0;
uint32_t screen_msg_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void update_display(void);
void manage_timers(void);
void process_key(uint8_t key);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    char key = keypad_scan(&keypad, GPIO_Pin);
    if (key != '\0') {
        ring_buffer_write(&keypad_rb, (uint8_t)key);
    }
}

/**
 * @brief Actualiza la pantalla OLED según el estado actual del sistema.
 */
void update_display(void) {
    ssd1306_Fill(Black); // Limpiar el buffer de la pantalla

    switch(current_screen_state) {
        case SCREEN_STATE_ENTER_PIN:
            ssd1306_SetCursor(18, 10);
            ssd1306_WriteString("Ingrese PIN:", Font_7x10, White);
            
            // ====================== SECCIÓN MODIFICADA ======================
            // En lugar de crear un string de asteriscos, ahora dibujamos
            // directamente la variable 'entered_password' para ver los dígitos.
            ssd1306_SetCursor(40, 32);
            ssd1306_WriteString(entered_password, Font_11x18, White);
            // ==================== FIN DE LA SECCIÓN MODIFICADA ================
            
            break;

        case SCREEN_STATE_GRANTED:
            ssd1306_SetCursor(5, 25);
            ssd1306_WriteString("Acceso Concedido", Font_7x10, White);
            break;

        case SCREEN_STATE_DENIED:
            ssd1306_SetCursor(5, 25);
            ssd1306_WriteString("Acceso Denegado", Font_7x10, White);
            break;
    }

    ssd1306_UpdateScreen(); // Enviar el buffer a la pantalla física
}

void process_key(uint8_t key) {
    if (current_screen_state != SCREEN_STATE_ENTER_PIN) {
        return;
    }

    led_on(&led1);
    led_timer_start = HAL_GetTick();
    led_on_duration = FEEDBACK_LED_TIME_MS;

    if (password_index < PASSWORD_LEN) {
        entered_password[password_index++] = (char)key;
    }

    if (password_index == PASSWORD_LEN) {
        if (strncmp(entered_password, PASSWORD, PASSWORD_LEN) == 0) {
            current_screen_state = SCREEN_STATE_GRANTED;
            led_on(&led1);
            led_timer_start = HAL_GetTick();
            led_on_duration = SUCCESS_LED_TIME_MS;
        } else {
            current_screen_state = SCREEN_STATE_DENIED;
            led_off(&led1);
            led_timer_start = 0;
        }
        screen_msg_timer = HAL_GetTick();
        
        password_index = 0;
        memset(entered_password, 0, sizeof(entered_password));
    }
    
    update_display();
}

void manage_timers(void) {
    if (led_timer_start != 0 && (HAL_GetTick() - led_timer_start > led_on_duration)) {
        led_off(&led1);
        led_timer_start = 0;
    }
    
    if (screen_msg_timer != 0 && (HAL_GetTick() - screen_msg_timer > MSG_DISPLAY_TIME_MS)) {
        current_screen_state = SCREEN_STATE_ENTER_PIN;
        screen_msg_timer = 0;
        update_display();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  led_init(&led1);
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN);
  keypad_init(&keypad);
  
  if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 100) == HAL_OK)
  {
      ssd1306_Init();
      update_display();
  }
  else
  {
      led_on(&led1);
      while(1);
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t key_from_buffer;
    
    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) {
        if (HAL_GetTick() - last_key_press_time > DEBOUNCE_TIME_MS) {
            last_key_press_time = HAL_GetTick();
            process_key(key_from_buffer);
        }
    }
    
    manage_timers();
    
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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) { Error_Handler(); }
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|KEYPAD_R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin (LED) and Keypad Rows */
  GPIO_InitStruct.Pin = LD2_Pin|KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* ====================== SECCIÓN CORREGIDA ====================== */
  /* Configure Keypad Columns as Inputs with Interrupts              */
  /* Cada puerto debe ser inicializado en su propia llamada.         */

  /* Configure Keypad Columns on GPIOA (C2, C3) */
  GPIO_InitStruct.Pin = KEYPAD_C2_Pin|KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Configure Keypad Column on GPIOB (C1) */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure Keypad Column on GPIOC (C4) */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* ==================== FIN DE LA SECCIÓN CORREGIDA ==================== */


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */