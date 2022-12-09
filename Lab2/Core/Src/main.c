/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

#define UART_DATA_RECEIVED (1UL << 0)
#define UART_DATA_ERROR    (1UL << 1)
#define UART_PRINTED       (1UL << 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes =
    {
        .name = "Task1",
        .stack_size = 128 * 4,
        .priority =
            (osPriority_t)osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes =
    {
        .name = "Task2",
        .stack_size = 128 * 4,
        .priority =
            (osPriority_t)osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes =
    {
        .name = "Task3",
        .stack_size = 128 * 4,
        .priority =
            (osPriority_t)osPriorityNormal,
};
/* Definitions for UARTdataQueue */
osMessageQueueId_t UARTdataQueueHandle;
const osMessageQueueAttr_t UARTdataQueue_attributes =
    {.name = "UARTdataQueue"};
/* Definitions for UARTFlags */
osEventFlagsId_t UARTFlagsHandle;
const osEventFlagsAttr_t UARTFlags_attributes =
    {.name = "UARTFlags"};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
void readUART(void *argument);
void printUART(void *argument);
void controlLEDs(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_UART4_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of UARTdataQueue */
    UARTdataQueueHandle = osMessageQueueNew(4, sizeof(uint8_t),
                                            &UARTdataQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of Task1 */
    Task1Handle = osThreadNew(readUART, NULL, &Task1_attributes);

    /* creation of Task2 */
    Task2Handle = osThreadNew(printUART, NULL, &Task2_attributes);

    /* creation of Task3 */
    Task3Handle = osThreadNew(controlLEDs, NULL, &Task3_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the event(s) */
    /* creation of UARTFlags */
    UARTFlagsHandle = osEventFlagsNew(&UARTFlags_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
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
    RCC_OscInitTypeDef RCC_OscInitStruct =
        {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
        {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
    GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, sizeof(ch), HAL_MAX_DELAY);
    return ch;
}
GETCHAR_PROTOTYPE
{
    uint8_t ch = 0;

    /* Clear the Overrun flag just before receiving the first character */
    __HAL_UART_CLEAR_OREFLAG(&huart4);

    /* Wait for reception of a character on the USART RX line and echo this
     * character on console */
    HAL_UART_Receive(&huart4, (uint8_t *)&ch, sizeof(ch), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, sizeof(ch), HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_readUART */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_readUART */
void readUART(void *argument)
{
    /* USER CODE BEGIN 5 */
    char buffer[4];
    /* Infinite loop */
    for (;;) {
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        HAL_UART_Receive(&huart4, (uint8_t *)&buffer, sizeof(buffer), HAL_MAX_DELAY);
        osMessageQueuePut(UARTdataQueueHandle, buffer, 0, 0);
        osEventFlagsSet(UARTFlagsHandle, UART_DATA_RECEIVED);
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_printUART */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_printUART */
void printUART(void *argument)
{
    /* USER CODE BEGIN printUART */
    /* Infinite loop */
    for (;;) {
        if (osEventFlagsWait(UARTFlagsHandle, UART_DATA_RECEIVED, osFlagsWaitAny, 500) == UART_DATA_RECEIVED) {
            char buffer[4];
            osMessageQueueGet(UARTdataQueueHandle, buffer, NULL, 0);
            printf("%s\n", buffer);
            osMessageQueuePut(UARTdataQueueHandle, buffer, 0, 0);
        } else {
            printf("%s\n", "Program running...");
            osDelay(500);
        }
    }
    /* USER CODE END printUART */
}

/* USER CODE BEGIN Header_controlLEDs */
/**
 * @brief Function implementing the Task3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_controlLEDs */
void controlLEDs(void *argument)
{
    /* USER CODE BEGIN controlLEDs */
    char buffer[4];
    /* Infinite loop */
    for (;;) {
        osEventFlagsWait(UARTFlagsHandle, UART_PRINTED, osFlagsWaitAny,
                         osWaitForever);
        osMessageQueueGet(UARTdataQueueHandle, buffer, NULL, 0);
        for (int i = 0; i < 4; i++) {
            if (buffer[i] != '1' || buffer[i] != '0') {
                osEventFlagsSet(UARTFlagsHandle, UART_DATA_ERROR);
            }
        }
        if (osEventFlagsWait(UARTFlagsHandle, UART_DATA_ERROR, osFlagsWaitAny, 50) != UART_DATA_ERROR) {
            HAL_GPIO_WritePin(GPIOB, LED1_Pin, buffer[0] - 48);
            HAL_GPIO_WritePin(GPIOB, LED2_Pin, buffer[1] - 48);
            HAL_GPIO_WritePin(GPIOB, LED3_Pin, buffer[2] - 48);
            HAL_GPIO_WritePin(GPIOB, LED4_Pin, buffer[3] - 48);
        } else {
            printf("%s\n", "Input error");
        }
        osDelay(1);
    }
    /* USER CODE END controlLEDs */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
