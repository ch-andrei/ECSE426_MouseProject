/**  
  ******************************************************************************
  * @file    accelerometer.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   Implementation for a means of reading the accelerometer XYZ values using NVIC interrupts
  */
	
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "supporting_functions.h"
#include "data.h"
#include "lis3dsh.h"
#include "accelerometer.h"
#include "accel_thread.h"
#include "kalman.h"

// pointer to the acceleration readings
float accel_data[3];

// kalman states
kalman_state_1d ksx, ksy, ksz;

// pointer to the acceleration readings
float accel[3];

void accelerometer_read(void){
	LIS3DSH_ReadACC(accel);
}

/**
   * @brief Updates the accelerometer's readings (includes calibration and filtering)
   * @retval None
   */
void accelerometer_update(void){
	accelerometer_read();
	accel_data[0] = (ACC11 * accel[0] + ACC12 * accel[1] + ACC13 * accel[2] + ACC10)/1000;
	accel_data[1] = (ACC21 * accel[0] + ACC22 * accel[1] + ACC23 * accel[2] + ACC20)/1000;
	accel_data[2] = (ACC31 * accel[0] + ACC32 * accel[1] + ACC33 * accel[2] + ACC30)/1000;
	Kalmanfilter_C_1d(accel_data[0], accel_data, &ksx);
	Kalmanfilter_C_1d(accel_data[1], accel_data + 1, &ksy);
	Kalmanfilter_C_1d(accel_data[2], accel_data + 2, &ksz);
}

/**
   * @brief Converts angle from radian to degrees and clamps to 0-180 range
	 * @param  None
   * @retval angle
   */
float convert_and_clamp_angle(float angle){
	angle = angle * 180 / acos(-1.0) + 90; // convert from rad to degrees and shift 
	angle = (angle > 180) ? 180 : angle; // clamp
	angle = (angle < 0) ? 0 : angle; // clamp
	return angle;
}

/**
   * @brief Computes the board's roll angle
	 * @param  None
   * @retval angle
   */
float accelerometer_get_roll_angle(void){
	float angle = (float) atan (accel_data[0] / sqrt( accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2] ));
	return convert_and_clamp_angle(angle);
}

/**
   * @brief Computes the board pitch angle
	 * @param  None
   * @retval angle
   */
float accelerometer_get_pitch_angle(void){
	float angle = (float) atan (accel_data[1] / sqrt( accel_data[0] * accel_data[0] + accel_data[2] * accel_data[2] ));
	return convert_and_clamp_angle(angle);
}

// init kalman filter states
void accel_kalman_config(void){
	init_kalman_1d(&ksx, 0.001, 0.07);
	init_kalman_1d(&ksy, 0.001, 0.07);
	init_kalman_1d(&ksz, 0.001, 0.07);
}

/**
   * @brief Configures accelerometer
   * @retval None
   */
void accelerometer_config(void){
	accel_kalman_config();
	
	LIS3DSH_InitTypeDef LIS3DSH_Ini;
	LIS3DSH_DRYInterruptConfigTypeDef LIS3DSH_InterruptConfig;
	SPI_HandleTypeDef SPI_Handle;
	
	LIS3DSH_Ini.Axes_Enable = LIS3DSH_XYZ_ENABLE;
	LIS3DSH_Ini.Continous_Update = LIS3DSH_ContinousUpdate_Enabled;
	LIS3DSH_Ini.Full_Scale = LIS3DSH_FULLSCALE_2;
	LIS3DSH_Ini.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_25;
	LIS3DSH_Init(&LIS3DSH_Ini);
	
	LIS3DSH_InterruptConfig.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;
	LIS3DSH_InterruptConfig.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;
	LIS3DSH_InterruptConfig.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;
	LIS3DSH_DataReadyInterruptConfig(&LIS3DSH_InterruptConfig);
	
	HAL_SPI_MspInit(&SPI_Handle);
	
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
}

/**
   * @brief Hardware interupt indicated that the accelerometer values are ready
   * @param  GPIO_Pin: Pin for interrupt signal
   * @retval None
   */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		EXTI0_IRQHandler();
		Accel_interrupt_handler();
	}
}
