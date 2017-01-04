/**  
  ******************************************************************************
  * @file    kalman.h
  * @author  Andrei Chubarau, Dmytry Grudin (McGill University)
  * @date    4-October-2016, 
  * @brief   header file for functions from kalman.c
  */
	
/* macro guard */
#ifndef KALMAN_H
#define KALMAN_H

/** @defgroup Kalman state initialization values define
  * @brief initial values kalman state struct will be loaded with
  * @{
  */
#define Q_init 0.01;
#define R_init 0.50;
#define X_init 0;
#define P_init 0.1;
#define K_init 0;
#define F_init 1;
#define H_init 1;

/*
* kalman state representation
*/
typedef struct kalman_state_1d
{
	float q;
	float r;
	float x;
	float p;
	float k;
	float f;
	float h;
} kalman_state_1d;

/* 
* initialize the values of kalman state.
*/
void init_kalman_1d(kalman_state_1d* ks, float Q, float R);

/* 
* performs 1d Kalman filtering on an input.
*/
int Kalmanfilter_C_1d(float input, float* output, kalman_state_1d* ks);

#endif
