
#include "Arduino.h"
#include "device.h"
#include "command_handler.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_wwdg.h"
#include "stm32f1xx_hal_iwdg.h"
#include "stm32f1xx_hal_flash_ex.h"

CommandHandler cmd_processor;

#define HW_FAULT_BLINK_DELAY_MS (200)

void HWFaultBlink(void)
{
    while (1)
    {
        digitalWrite(LED_SYS_PWM, HIGH);
        delay(HW_FAULT_BLINK_DELAY_MS);
        digitalWrite(LED_SYS_PWM, LOW);
        delay(HW_FAULT_BLINK_DELAY_MS);
    }
}

void NMI_Handler(void) {
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    HWFaultBlink();
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  HWFaultBlink();
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  HWFaultBlink();
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  HWFaultBlink();
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  HWFaultBlink();
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
  HWFaultBlink();
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
  HWFaultBlink();
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
  HWFaultBlink();
}

void WWDG_IRQHandler(void)
{
  HWFaultBlink();
}

void IWDG_IRQHandler(void)
{
  HWFaultBlink();
}

void PVD_IRQHandler(void)
{
  HWFaultBlink();
}

void TAMPER_IRQHandler(void)
{
  HWFaultBlink();
}

void SystemClock_Config(void)
{
  init_device();
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  
  //__HAL_DBGMCU_FREEZE_IWDG();
  //__HAL_DBGMCU_FREEZE_WWDG();
  FLASH_OBProgramInitTypeDef option_bytes = {0};
  HAL_FLASHEx_OBGetConfig(&option_bytes);
  if (!(option_bytes.USERConfig & OB_IWDG_SW)) {
      HAL_FLASH_Unlock();
      HAL_FLASH_OB_Unlock();
      HAL_FLASHEx_OBErase();
      option_bytes.OptionType = OPTIONBYTE_USER;
      option_bytes.USERConfig |= OB_IWDG_SW;
      HAL_FLASHEx_OBProgram(&option_bytes);
      HAL_FLASH_OB_Lock();
      HAL_FLASH_Lock();
      HAL_FLASH_OB_Launch(); // Triggers reset
  }
}

void setup() {
  //WWDG->CR &= ~(1UL << WWDG_CR_WDGA_Pos);
}

void loop() {
  cmd_processor.run();
  //WWDG->CR |= WWDG_CR_T_Msk; // Refresh WWDG
  //IWDG->KR = IWDG_KEY_RELOAD; // Refresh IWDG
}
