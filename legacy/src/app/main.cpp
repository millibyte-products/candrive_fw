
#include "Arduino.h"
#include "device.h"
#include "motor.h"
#include "protocol.h"
#include "can_interface.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_wwdg.h"
#include "stm32f1xx_hal_iwdg.h"
#include "stm32f1xx_hal_flash_ex.h"

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

void NMI_Handler(void)
{
  HWFaultBlink();
}

void HardFault_Handler(void)
{
  HWFaultBlink();
}

void MemManage_Handler(void)
{

  HWFaultBlink();
}

void BusFault_Handler(void)
{

  HWFaultBlink();
}

void UsageFault_Handler(void)
{

  HWFaultBlink();
}

void SVC_Handler(void)
{
  HWFaultBlink();
}

void DebugMon_Handler(void)
{
  HWFaultBlink();
}

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
  device_init();
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  if (!(option_bytes.USERConfig & OB_IWDG_SW))
  {
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

void setup()
{
  device_init();
  motor_init();
  can_init();
  reset_interfaces();
}

void loop()
{
  can_read();
  motor_update();
  device_update();
}

/*
#include "Arduino.h"
#include "SPI.h"
#define MT6701_SSI_CLOCK 1000000
#include <MT6701.h>

#define CS_PIN PA4

MT6701 encoder;
HardwareSerial Serial3(PB11, PB10);

static uint8_t i = 0;

void setup() {
  Serial3.begin(115200);
  Serial3.println("Encoder init");
  SPI.begin();
  encoder.initializeSSI(CS_PIN);
}

void loop() {

  // Set work angles
  // Combine with offsetSet() for better range selection
  Serial3.printf("%02x", i);
  Serial3.print("\t Angle: ");
  Serial3.print(encoder.angleRead());
  Serial3.print("\t Field: ");
  Serial3.print(encoder.fieldStatusRead());
  Serial3.print("\r");
  i += 1;
}
*/
