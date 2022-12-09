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
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACTIVATE_FLAG (1UL << 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* Definitions for Tester */
osThreadId_t TesterHandle;
const osThreadAttr_t Tester_attributes = {
    .name       = "Tester",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for ECU */
osThreadId_t ECUHandle;
const osThreadAttr_t ECU_attributes = {
    .name       = "ECU",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for DoneTransfer */
osEventFlagsId_t DoneTransferHandle;
const osEventFlagsAttr_t DoneTransfer_attributes = {
    .name = "DoneTransfer"};
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef Node1_TxHeader;
CAN_RxHeaderTypeDef Node1_RxHeader;

CAN_TxHeaderTypeDef Node2_TxHeader;
CAN_RxHeaderTypeDef Node2_RxHeader;

uint32_t Node1TxMsgMailBox;
uint32_t Node2TxMsgMailBox;

uint8_t Node1_TxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
void TesterTask(void *argument);
void ECUTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    Node1_TxHeader.IDE   = CAN_ID_STD;
    Node1_TxHeader.StdId = 0x712;
    Node1_TxHeader.RTR   = CAN_RTR_DATA;
    Node1_TxHeader.DLC   = 8;

    Node2_TxHeader.IDE   = CAN_ID_STD;
    Node2_TxHeader.StdId = 0x7A2;
    Node2_TxHeader.RTR   = CAN_RTR_DATA;
    Node2_TxHeader.DLC   = 8;
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
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
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

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of Tester */
    TesterHandle = osThreadNew(TesterTask, NULL, &Tester_attributes);

    /* creation of ECU */
    ECUHandle = osThreadNew(ECUTask, NULL, &ECU_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the event(s) */
    /* creation of DoneTransfer */
    DoneTransferHandle = osEventFlagsNew(&DoneTransfer_attributes);

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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 72;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig           = {0};
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution            = ADC_RESOLUTION_8B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel      = ADC_CHANNEL_3;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel               = ADC_CHANNEL_3;
    sConfigInjected.InjectedRank                  = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedNbrOfConversion       = 1;
    sConfigInjected.InjectedSamplingTime          = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInjected.ExternalTrigInjecConv         = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv              = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset                = 0;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 9;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_3TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_4TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */
    CAN_FilterTypeDef canfilterconfig;
    canfilterconfig.FilterActivation     = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank           = 18;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh         = 0x7A2 << 5;
    canfilterconfig.FilterIdLow          = 0;
    canfilterconfig.FilterMaskIdHigh     = 0x7A2 << 5;
    canfilterconfig.FilterMaskIdLow      = 0x0000;
    canfilterconfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 20;

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
    HAL_CAN_Start(&hcan1);
    /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

    /* USER CODE BEGIN CAN2_Init 0 */

    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance                  = CAN2;
    hcan2.Init.Prescaler            = 9;
    hcan2.Init.Mode                 = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth        = CAN_SJW_2TQ;
    hcan2.Init.TimeSeg1             = CAN_BS1_3TQ;
    hcan2.Init.TimeSeg2             = CAN_BS2_4TQ;
    hcan2.Init.TimeTriggeredMode    = DISABLE;
    hcan2.Init.AutoBusOff           = DISABLE;
    hcan2.Init.AutoWakeUp           = DISABLE;
    hcan2.Init.AutoRetransmission   = DISABLE;
    hcan2.Init.ReceiveFifoLocked    = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */
    CAN_FilterTypeDef canfilterconfig;
    canfilterconfig.FilterActivation     = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank           = 10;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    canfilterconfig.FilterIdHigh         = 0x712 << 5;
    canfilterconfig.FilterIdLow          = 0;
    canfilterconfig.FilterMaskIdHigh     = 0x712 << 5;
    canfilterconfig.FilterMaskIdLow      = 0;
    canfilterconfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
    HAL_CAN_Start(&hcan2);
    /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
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
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : Button_Pin */
    GPIO_InitStruct.Pin  = Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
    GPIO_InitStruct.Pin   = LD1_Pin | LD3_Pin | LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_OverCurrent_Pin */
    GPIO_InitStruct.Pin  = USB_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, sizeof(ch) - 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_TesterTask */
/**
 * @brief  Function implementing the Tester thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_TesterTask */
void TesterTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    // khai báo các mảng dữ liệu
    uint8_t const middle[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t const left[10]   = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    uint8_t const right[10]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t const DataID[2]  = {0xF0, 0x02};
    GPIO_PinState a;
    uint8_t message1[8] = {};
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        a = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
        if (a == GPIO_PIN_SET) {
            // First Frame
            message1[0] = 0x10;
            message1[1] = 0x0A;
            message1[2] = 0x2E;
            message1[3] = DataID[0];
            message1[4] = DataID[1];
            for (int i = 5; i < 8; i++) {
                message1[i] = middle[i - 5];
            }
            HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, message1, &Node1TxMsgMailBox);
            message1[0] = 0x21;
            for (int i = 1; i < 8; i++) {
                message1[i] = middle[i + 2];
            }
            HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, message1, &Node1TxMsgMailBox);
        } else {
            // First Frame
            message1[0] = 0x10;
            message1[1] = 0x0A;
            message1[2] = 0x2E;
            message1[3] = DataID[0];
            message1[4] = DataID[1];
            for (int i = 5; i < 8; i++) {
                message1[i] = right[i - 5];
            }
            HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, message1, &Node1TxMsgMailBox);
            message1[0] = 0x21;
            for (int i = 1; i < 8; i++) {
                message1[i] = right[i + 2];
            }
            HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, message1, &Node1TxMsgMailBox);
        }
        osEventFlagsSet(DoneTransferHandle, ACTIVATE_FLAG);
        osDelay(1000);
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Node1_RxHeader, message1);
        if (message1[0] == 0x2E + 0x40) {
            printf("%s\n", "Success");
        }
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ECUTask */
/**
 * @brief Function implementing the ECU thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ECUTask */
void ECUTask(void *argument)
{
    /* USER CODE BEGIN ECUTask */
    uint32_t fifoLevel;
    uint8_t raw_message[8] = {};
    uint8_t message2[30]   = {};
    uint32_t datalength;
    uint8_t UDS_service_ID;
    uint8_t DataID[2];
    uint8_t reponse[3];
    int i;
    /* Infinite loop */
    for (;;) {
        osEventFlagsWait(DoneTransferHandle, ACTIVATE_FLAG, osFlagsWaitAny, osWaitForever);
        fifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1);
        if (fifoLevel != 0) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &Node2_RxHeader, raw_message);
            if (raw_message[0] & 0b11110000) {
                datalength     = raw_message[0] & 0b00001111;
                datalength     = (datalength << 4) + raw_message[1];
                UDS_service_ID = raw_message[2];
                DataID[0]      = raw_message[3];
                DataID[1]      = raw_message[4];
                for (i = 0; i + 5 < 8; i++) {
                    message2[i] = raw_message[i + 5];
                }
                for (i = i; i < datalength; i++) {
                    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &Node2_RxHeader, raw_message);
                    for (int x = 1; x < 8; x++) {
                        message2[i] = raw_message[x];
                        i++;
                        if (i == datalength) break;
                    }
                }
            } else {
                datalength = raw_message[0] & 0b00001111;
                for (i = 0; i < datalength; i++) {
                    message2[i] = raw_message[i + 1];
                }
            }
            fifoLevel = 0;
            for (int x = 0; x < datalength; x++) {
                printf("%02X ", message2[x]);
            }
            printf("\n");
            Node2_TxHeader.DLC = 3;
            reponse[0]         = UDS_service_ID + 0x40;
            reponse[1]         = DataID[0];
            reponse[2]         = DataID[1];
            HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, reponse, &Node2TxMsgMailBox);
        }
    }
    /* USER CODE END ECUTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
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
