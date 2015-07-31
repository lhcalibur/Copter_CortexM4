#include "stm32f4xx_nucleo.h"

#include "error_handler.h"

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Blink LED */
  while(1)
  {
	  BSP_LED_Toggle(LED2);
	  HAL_Delay(100);
  }
}
