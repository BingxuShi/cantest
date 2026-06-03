#ifndef EC11_H
#define EC11_H


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "gpio.h"


#define EC11_PH_A HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define EC11_PH_B HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define _BV(n) (1<<(n))
#define ENCODER_PULSES_PER_STEP 4

void EC11_init(void);
void EC11_timer(void);
void EC11_update(void);

#endif
