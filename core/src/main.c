/// @file main.c
/// @brief 
/// @attention

#include "main.h"
#include "mylibrary.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_flash.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_DELAY(1000);

  HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, 2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Hello World\n", 12, 1000);

  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim2);
  memset(ADC_Values,0, sizeof(ADC_Values)); 

  while (1)
  {
    if(send_flag == 1){
        send_flag = 0;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,RESET);
        char sendBuffer[350];
        memset(sendBuffer,0,sizeof(sendBuffer));
        sprintf(sendBuffer, "ADC1: %d, ADC2: %d\n", ADC_Values[0], ADC_Values[1]);
        if (stream_index == 0){
            for(int i=0; i<64;i++){
                itoa(ADC_Values[3][i], &sendBuffer[strlen(sendBuffer)], 10);
                sendBuffer[strlen(sendBuffer)] = ',';

            }
        }
        else{
            for(int i=0; i<64;i++){
                itoa(ADC_Values[stream_index-1][i], &sendBuffer[strlen(sendBuffer)], 10);
                sendBuffer[strlen(sendBuffer)] = ',';
            }
        }
        sendBuffer[strlen(sendBuffer)-1] = '\n';
        HAL_UART_Transmit(&huart2, (uint8_t*)sendBuffer, strlen(sendBuffer), 200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,SET);
    }
  }
}
/// @brief : System Clock Configuration
/// @retval 

void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
        Error_Handler();
    } 

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (CC_CLockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) !=HAL_OK){
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        Error_Handler();
    }
}

/// @brief : ADC1 Initialization Function
/// @param : None

static void MX_ADC1_Init(void){
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK){
        Error_Handler();
    }
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
        Error_Handler();
    }
}

    /// @brief : TIM2 Initialization Function
    /// @param : None 
    /// @retval: None
static void MX_TIM2_Init(void){

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1033;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1088;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK){
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK){
        Error_Handler();
    }
}

    /// @brief : UART2 Initialization Function
    /// @param : None 
    /// @retval: None
static void MX_UART2_UART_Init(void){
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK){
        Error_Handler();
    }
}

static void MX_DMA_Init(void){
    HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

    ///  @brief :GPIO Initialization Function
    ///  @param : None
    ///   @retval: None

static void MX_GPIO_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

/// GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/// @brief : This function is executed in case of error occurrence.
/// @retval : None

void Error_Handler(void){


}
#ifdef USE_FULL_ASSERT
/// @brief : Reports the name of the source file and the source line number where the assert_param error has occurred.
/// @param file: pointer to the source file name
/// @param file: assert_param error line source number
/// @retval : None

void assert_failed(uint8_t *file, uint32_t line){

}
#endif
