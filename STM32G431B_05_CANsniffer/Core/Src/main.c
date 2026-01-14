/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CAN Monitor - Receives CAN messages and logs them via UART
  * @description    : This program configures an STM32G431 as a CAN bus monitor.
  *                   It receives all CAN messages on the bus and prints them
  *                   to the serial terminal (UART2) with ID, DLC, and data bytes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   // For snprintf() formatting
#include <string.h>  // For strlen() in UART transmit
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;  // FDCAN1 peripheral handle

UART_HandleTypeDef huart2;    // USART2 peripheral handle for debug output

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Buffer for UART printf-style output (200 bytes max)
char uart_buf[200];

/**
  * @brief  Sends a string to UART2 for serial terminal output
  * @param  msg: Pointer to null-terminated string to transmit
  * @retval None
  */
void uart_print(char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// CAN filter configuration structure
FDCAN_FilterTypeDef sFilterConfig;

// CAN receive header (contains ID, DLC, timestamp, etc.)
FDCAN_RxHeaderTypeDef RxHeader;

// Buffer to store received CAN data bytes (max 8 bytes for Classic CAN)
uint8_t RxData[8];

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
  MX_GPIO_Init();         // Initialize GPIO pins (LED, button, CAN_TERMINAL)
  MX_FDCAN1_Init();       // Initialize FDCAN1 peripheral
  MX_USART2_UART_Init();  // Initialize USART2 for debug output
  
  /* USER CODE BEGIN 2 */

  // Disable CAN termination resistor (120Ω)
  // For point-to-point connection, termination is not needed
  HAL_GPIO_WritePin(CAN_TERMINAL_GPIO_Port, CAN_TERMINAL_Pin, GPIO_PIN_RESET);

  // Configure CAN filter to accept ALL messages (ID 0x000 to 0x7FF)
  sFilterConfig.IdType = FDCAN_STANDARD_ID;           // Use 11-bit Standard IDs
  sFilterConfig.FilterIndex = 0;                      // Use filter bank 0
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;      // Range filter mode
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route to RX FIFO 0
  sFilterConfig.FilterID1 = 0x000;  // Minimum ID to accept
  sFilterConfig.FilterID2 = 0x7FF;  // Maximum ID to accept (all Standard IDs)

  // Wait 3 seconds for UART terminal to connect
  HAL_Delay(3000);

  // Apply the filter configuration
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
      uart_print("ERROR: Filter config failed!\r\n");
      Error_Handler();
  }

  // Configure global filter to accept Standard and Extended IDs
  // This works in conjunction with the specific filter configured above
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,  // Accept Standard IDs to FIFO0
                                    FDCAN_ACCEPT_IN_RX_FIFO0,  // Accept Extended IDs to FIFO0
                                    FDCAN_FILTER_REMOTE,       // Remote frames handling
                                    FDCAN_FILTER_REMOTE) != HAL_OK) {
      uart_print("ERROR: Global filter config failed!\r\n");
      Error_Handler();
  }

  // Enable interrupt notification for new messages in RX FIFO0
  // This triggers HAL_FDCAN_RxFifo0Callback() when a message is received
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      0) != HAL_OK) {
      uart_print("ERROR: Notification activation failed!\r\n");
      Error_Handler();
  }

  // Manually enable FDCAN interrupts in NVIC (if not done in CubeMX)
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 4, 0);  // Set priority 4
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);          // Enable interrupt line 0
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 4, 0);  // Set priority 4
  HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);          // Enable interrupt line 1

  // Re-activate notification (redundant but safe)
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      0) != HAL_OK) {
      uart_print("ERROR: Notification activation failed!\r\n");
      Error_Handler();
  }

  // Start the FDCAN peripheral - CAN bus is now active
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
      uart_print("ERROR: FDCAN start failed!\r\n");
      Error_Handler();
  }

  // Print startup message to serial terminal
  uart_print("\r\n=== CAN Monitor Started ===\r\n");
  uart_print("Waiting for CAN messages...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Main loop does nothing - all CAN reception is handled in interrupt callback
    HAL_Delay(1000);
    // Optional: Uncomment below for heartbeat indicator on terminal
    //uart_print(".");  // Heartbeat every second

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @description Configures system clock to 160 MHz using HSE (8 MHz crystal)
  *              HSE -> PLL (/2, x80, /2) -> 160 MHz SYSCLK
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Use external crystal
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    // Enable HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        // PLL source = HSE
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;                 // Divider M = 2 (8MHz/2 = 4MHz)
  RCC_OscInitStruct.PLL.PLLN = 80;                            // Multiplier N = 80 (4MHz*80 = 320MHz)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                 // P divider (not used)
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                 // Q divider (not used)
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                 // R divider = 2 (320MHz/2 = 160MHz SYSCLK)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // System clock from PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          // AHB = SYSCLK (160 MHz)
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;           // APB1 = HCLK (160 MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           // APB2 = HCLK (160 MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  * @description Configures FDCAN1 for Classic CAN at 1 Mbit/s
  *              Bit timing: Prescaler=16, Seg1=7, Seg2=2 -> 1 Mbit/s @ 160MHz
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;                                   // Use FDCAN1 peripheral
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;                // No clock division
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;              // Classic CAN (not CAN-FD)
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;                       // Normal operation mode
  hfdcan1.Init.AutoRetransmission = ENABLE;                    // Auto-retransmit on error
  hfdcan1.Init.TransmitPause = DISABLE;                        // No transmit pause
  hfdcan1.Init.ProtocolException = DISABLE;                    // Protocol exception handling disabled
  
  // Nominal bit timing configuration for 1 Mbit/s @ 160 MHz
  // Bit time = (1 + NominalTimeSeg1 + NominalTimeSeg2) = 10 TQ (Time Quanta)
  // TQ = NominalPrescaler / FDCAN_CLK = 16 / 160MHz = 100 ns
  // Bit rate = 1 / (10 * 100ns) = 1 Mbit/s
  hfdcan1.Init.NominalPrescaler = 16;                          // Prescaler = 16
  hfdcan1.Init.NominalSyncJumpWidth = 1;                       // SJW = 1 TQ
  hfdcan1.Init.NominalTimeSeg1 = 7;                            // Seg1 = 7 TQ (sample point at 80%)
  hfdcan1.Init.NominalTimeSeg2 = 2;                            // Seg2 = 2 TQ
  
  // Data bit timing (not used for Classic CAN, only for CAN-FD)
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  
  // Filter configuration
  hfdcan1.Init.StdFiltersNbr = 0;                              // Number of standard filters (configured later)
  hfdcan1.Init.ExtFiltersNbr = 0;                              // Number of extended filters
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;      // TX FIFO mode
  
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  * @description Configures USART2 for 115200 baud, 8N1, no flow control
  *              Used for debug output to serial terminal
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;                                    // Use USART2 peripheral
  huart2.Init.BaudRate = 115200;                               // 115200 bits per second
  huart2.Init.WordLength = UART_WORDLENGTH_8B;                 // 8 data bits
  huart2.Init.StopBits = UART_STOPBITS_1;                      // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;                       // No parity
  huart2.Init.Mode = UART_MODE_TX_RX;                          // TX and RX enabled
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;                 // No hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;             // 16x oversampling
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;    // 3-bit sample
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;            // No prescaler
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features
  
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Configure TX FIFO threshold (not critical for this application)
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Configure RX FIFO threshold (not critical for this application)
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Disable FIFO mode for simpler operation
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  * @description Configures GPIO pins:
  *              - PC13: CAN_TERMINAL (output, controls 120Ω termination resistor)
  *              - PC6:  USER_LED (output, for status indication)
  *              - PA4:  USER_BUTTON (input with pull-up)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();  // Enable GPIOC clock
  __HAL_RCC_GPIOF_CLK_ENABLE();  // Enable GPIOF clock (for HSE pins)
  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA clock (for UART, button)
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIOB clock (for CAN pins)

  /*Configure GPIO pin Output Level */
  // Initialize CAN_TERMINAL and USER_LED to LOW
  HAL_GPIO_WritePin(GPIOC, CAN_TERMINAL_Pin|USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_TERMINAL_Pin USER_LED_Pin */
  GPIO_InitStruct.Pin = CAN_TERMINAL_Pin|USER_LED_Pin;   // PC13 and PC6
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;             // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;                     // No pull-up/pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;            // Low speed (sufficient for LED/control)
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;                  // PA4
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                 // Input mode
  GPIO_InitStruct.Pull = GPIO_PULLUP;                     // Internal pull-up (button pulls to GND)
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  CAN RX FIFO 0 Callback
  * @param  hfdcan: pointer to FDCAN handle
  * @param  RxFifo0ITs: interrupt flags for RX FIFO 0
  * @retval None
  * @description This callback is triggered when a new CAN message arrives in RX FIFO 0.
  *              It reads the message, extracts ID and data, and prints to UART.
  *              Called automatically by HAL_FDCAN_IRQHandler() when interrupt occurs.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    
    // Check if the interrupt is for a new message in FIFO 0
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {

        // Read the message from RX FIFO 0
        // RxHeader will contain: ID, DLC, timestamp, filter index, etc.
        // RxData will contain the actual data bytes
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

            // Convert FDCAN DataLength enumeration to actual byte count
            // DataLength is NOT a simple number, but an enumerated constant
            // Must use switch statement to decode it properly
            uint8_t data_length = 0;
            switch (RxHeader.DataLength) {
                case FDCAN_DLC_BYTES_0:  data_length = 0; break;   // DLC 0
                case FDCAN_DLC_BYTES_1:  data_length = 1; break;   // DLC 1
                case FDCAN_DLC_BYTES_2:  data_length = 2; break;   // DLC 2
                case FDCAN_DLC_BYTES_3:  data_length = 3; break;   // DLC 3
                case FDCAN_DLC_BYTES_4:  data_length = 4; break;   // DLC 4
                case FDCAN_DLC_BYTES_5:  data_length = 5; break;   // DLC 5
                case FDCAN_DLC_BYTES_6:  data_length = 6; break;   // DLC 6
                case FDCAN_DLC_BYTES_7:  data_length = 7; break;   // DLC 7
                case FDCAN_DLC_BYTES_8:  data_length = 8; break;   // DLC 8
                
                // CAN-FD extended DLC values (not used in Classic CAN)
                case FDCAN_DLC_BYTES_12: data_length = 12; break;
                case FDCAN_DLC_BYTES_16: data_length = 16; break;
                case FDCAN_DLC_BYTES_20: data_length = 20; break;
                case FDCAN_DLC_BYTES_24: data_length = 24; break;
                case FDCAN_DLC_BYTES_32: data_length = 32; break;
                case FDCAN_DLC_BYTES_48: data_length = 48; break;
                case FDCAN_DLC_BYTES_64: data_length = 64; break;
                
                default: data_length = 0; break;  // Unknown DLC
            }

            // Format and print message ID and DLC
            // Example output: "[RX] ID: 0x123  DLC: 8  Data: "
            snprintf(uart_buf, sizeof(uart_buf),
                     "\r\n[RX] ID: 0x%03lX  DLC: %u  Data: ",
                     RxHeader.Identifier,  // CAN message ID (11-bit for Standard)
                     data_length);         // Number of data bytes
            uart_print(uart_buf);

            // Print each data byte in hexadecimal format
            // Example output: "11 22 33 44 55 66 77 88 "
            for (uint8_t i = 0; i < data_length; i++) {
                snprintf(uart_buf, sizeof(uart_buf), "%02X ", RxData[i]);
                uart_print(uart_buf);
            }
            
            // Print newline to end the message
            uart_print("\r\n");
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  * @description Disables interrupts and enters infinite loop.
  *              Can be used as breakpoint location for debugging.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();  // Disable all interrupts
  while (1)
  {
    // Stuck here forever - attach debugger to investigate
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
