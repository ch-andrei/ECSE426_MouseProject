/**  
  ******************************************************************************
  * @file    segments.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from segments.c
  */

#ifndef SEGMENTS_H
#define SEGMENTS_H

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"

#define NUMBER_OF_SEGMENTS 									4

#define TEMPERATURE_ALERT_ON								1
#define TEMPERATURE_ALERT_OFF								0

#define TEMPERATURE_ALERT_COUNTER_ALERT_ON	100
#define TEMPERATURE_ALERT_COUNTER_MODULO 		400

#define ALERT_TEMPERATURE_THRESHOLD 				35

/** @defgroup Digit_pins_define	Digits for 7-segment display pins define
  * @brief GPIO Port C pins
  * @{
  */
// 3 digits with degree symbol for 7-segment display (PORT C)
#define TENS		GPIO_PIN_0
#define ONES		GPIO_PIN_1
#define DECIMAL	GPIO_PIN_2
#define	DEGREE	GPIO_PIN_3

/** @defgroup Segment_pins_define	7-segment pins define (PORT E) define
  * @brief GPIO Port E pins
  * @{
  */
#define	A		GPIO_PIN_7
#define	B		GPIO_PIN_8
#define	C		GPIO_PIN_9
#define D		GPIO_PIN_10
#define E		GPIO_PIN_11
#define F		GPIO_PIN_12
#define G		GPIO_PIN_13
#define DP	GPIO_PIN_14

/**
   * @brief Configures GPIO for 7-segment display
   * @retval None
   */
void segments_configure(void);

/** @defgroup temperature_update_rate_define Temperature update rate define
  * @brief update rate for the value of the displayed temperature (update the value every SEGMENTS_VALUE_UPDATE_PERIOD calls to segments_update_display)
  * @{
  */
#define SEGMENTS_VALUE_UPDATE_PERIOD_DEFAULT 25

/**
   * @brief Updates the 4 segments display incrementally; must be called 4 times for a full update
   * @param temperature: current temperature of the chip
	 * @param piezo_strength: strength of piezo hit
   * @retval None
   */
void segments_update_display(float value);

void segments_force_display(float value);

#endif
