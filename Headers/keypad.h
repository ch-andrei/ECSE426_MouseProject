// macro guard
#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f4xx_hal.h"

// wait for KEYPAD_DEBOUNCE_TIME (milliseconds) when reading a key
#define KEYPAD_DEBOUNCE_TIME 10

/** @defgroup keypad GPIO pins
  * @brief from GPIO Port E
  * @{
  */
#define KEYPAD_GPIO_CLK_ENABLE	__HAL_RCC_GPIOE_CLK_ENABLE

#define KEYPAD_GPIOX GPIOE
#define	Col1		GPIO_PIN_10
#define	Col2		GPIO_PIN_11
#define	Col3		GPIO_PIN_12
#define Row1		GPIO_PIN_6
#define Row2		GPIO_PIN_7
#define Row3		GPIO_PIN_8
#define Row4		GPIO_PIN_9

#define DEBOUNCE_ON 	1
#define DEBOUNCE_OFF	0

// configure the keypad
void keypad_configure(void);
uint8_t keypad_read(void);

// handles key bouncing
uint8_t keypad_read_debounce(uint8_t *var, uint8_t debounce);

#endif
