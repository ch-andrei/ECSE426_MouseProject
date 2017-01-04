/**  
  ******************************************************************************
  * @file    accelerometer.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   Header file for accelerometer.c
  */
	
// macro guard
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

// calibration values
#define ACC11 0.978524626070986           
#define ACC21 -0.0224859720610610
#define ACC31 -0.0127494482535358
#define ACC12 0.00542193231501795
#define ACC22 1.02027872053166
#define ACC32 0.00502358267867257
#define ACC13 0.00914155751245639 
#define ACC23 0.00134221223879511 
#define ACC33 0.984603754591209

#define ACC10 -11.5809713807721
#define ACC20 -19.9642039453109
#define ACC30 -39.9651628742313

/**
   * @brief Configure MEMS, its interrupts, and the associated GPIO
   * @retval None
   */
void accelerometer_config(void);

/**
   * @brief Computes the board's roll angle
	 * @param  None
   * @retval angle
   */
float accelerometer_get_roll_angle(void);

/**
   * @brief Computes the board pitch angle
	 * @param  None
   * @retval angle
   */
float accelerometer_get_pitch_angle(void);

/**
   * @brief Updates the accelerometer's readings
   * @retval None
   */
void accelerometer_update(void);

#endif
