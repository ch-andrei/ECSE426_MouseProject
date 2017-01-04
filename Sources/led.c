/**  
  ******************************************************************************
  * @file    led.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   code to initialize and use the LED lights
  */

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
#include "led.h"
#include "data.h"
#include "main.h"

/**
   * @brief Initialize LEDs
   * @retval None
   */
void led_configure(void){
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitD;
	GPIO_InitD.Pin = LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE;
	GPIO_InitD.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitD.Pull = GPIO_NOPULL;
	GPIO_InitD.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitD);
}

/**
   * @brief Turn an LED on
	 * @param led number of the LED to turn on
   * @retval None
   */
void led_toggle_led_on(uint32_t led){
	HAL_GPIO_WritePin(GPIOD, led, GPIO_PIN_SET);
}

/**
   * @brief Turn an LED off
	 * @param led number of the LED to turn off
   * @retval None
   */
void led_toggle_led_off(uint32_t led){
	HAL_GPIO_WritePin(GPIOD, led, GPIO_PIN_RESET);
}
