/**  
  ******************************************************************************
  * @file    led.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from led.c
  */

/* macro guard */
#ifndef LED_H
#define LED_H

#include "stdint.h"

/** @defgroup LED_pins_define	LED pins define
  * @brief GIPO Port D pins
  * @{
  */
#define LED_GREEN 		GPIO_PIN_12
#define LED_ORANGE 		GPIO_PIN_13
#define LED_RED				GPIO_PIN_14
#define LED_BLUE			GPIO_PIN_15

/**
   * @brief Configures GPIO for LED
   * @retval None
   */
void led_configure(void);

/**
   * @brief Turns a specific LED on
   * @param LED pin
	 * @retval None
   */
void led_toggle_led_on(uint32_t led);

/**
   * @brief Turns a specific LED off
   * @param led: pin
	 * @retval None
   */
void led_toggle_led_off(uint32_t led);

#endif /* LED_H */
