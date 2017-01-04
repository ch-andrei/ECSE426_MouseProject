/**  
  ******************************************************************************
  * @file    kalman.c
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   Implementation of Kalman Filter in 1-dimension
  */
#include <stdio.h>
#include <stdlib.h>
#include <kalman.h>
#include <data.h>

/**
   * @brief initialize the values of kalman state.
   * @retval None
   */
void init_kalman_1d(kalman_state_1d* ks, float Q, float R)
{
	ks->q = Q;
	ks->r = R;
	ks->x = X_init;
	ks->p = P_init;
	ks->k = K_init;
	ks->f = F_init;
	ks->h = H_init;
}

/**
   * @brief performs 1d Kalman filtering on an input.
	 * @param input input float value
	 * @param output pointer to output float value
	 * @param ks pointer to kalman state struct
   * @retval None
   */
int Kalmanfilter_C_1d(float input, float* output, kalman_state_1d* ks)
{
	ks->x = ks->f * ks->x;
	ks->p = ks->f * ks->f * ks->p + ks->q;
	float temp = ks->h * ks->h * ks->p + ks->r;
	if (temp == 0) return -1;
	ks->k = ks->p * ks->h / temp;
	ks->p = ks->p - ks->p * ks->k * ks->h;
	ks->x = ks->x + ks->k * ( input - ks->h * ks->x );	
	*output = ks->x;
	return 0;
}
