
#ifndef TIMER_H
#define TIMER_H
 
/* Configuration ***********************************/
void timer_configure(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /*__LED_H */
