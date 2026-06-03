#ifndef __EC11_H
#define __EC11_H

#include "stm32f1xx_hal.h"

//????
#define EC11_SW_PIN    GPIO_PIN_0
#define EC11_SW_PORT   GPIOC
#define EC11_A_PIN     GPIO_PIN_1
#define EC11_A_PORT    GPIOC
#define EC11_B_PIN     GPIO_PIN_2
#define EC11_B_PORT    GPIOC

//?????
typedef struct
{
  int16_t cnt;        //??????
  uint8_t dir;        //1:?? -1:?? 0:??
  uint8_t key_flag;   //1????
}EC11_t;

extern EC11_t ec11;

void EC11_Scan(void); //1ms????
uint8_t EC11_GetKey(void);
int16_t EC11_GetCnt(void);
void EC11_ClearCnt(void);

#endif
