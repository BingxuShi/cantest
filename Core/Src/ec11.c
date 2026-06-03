#include "ec11.h"

//????
#define KEY_DELAY_MS 20U
#define ENCODER_FILTER_CNT 2U

EC11_t ec11;
static uint16_t key_tick = 0;
static uint8_t last_a = 0; //?????A???

void EC11_Scan(void)
{
  uint8_t now_a,now_b;
  //??AB?
  now_a = HAL_GPIO_ReadPin(EC11_A_PORT,EC11_A_PIN);
  now_b = HAL_GPIO_ReadPin(EC11_B_PORT,EC11_B_PIN);

  //================???????================
  if(last_a != now_a) //A?????
  {
    if(now_a == 0) //???????
    {
      if(now_b == 1)
      {
        ec11.cnt++;
        ec11.dir = 1;
      }
      else
      {
        ec11.cnt--;
        ec11.dir = -1;
      }
    }
  }
  last_a = now_a;

  //================???? ????================
  if(HAL_GPIO_ReadPin(EC11_SW_PORT,EC11_SW_PIN) == GPIO_PIN_RESET) //???????
  {
    key_tick++;
    if(key_tick >= KEY_DELAY_MS)
    {
      ec11.key_flag = 1;
      key_tick = KEY_DELAY_MS; //????
    }
  }
  else
  {
    key_tick = 0;
  }
}

//??????,??????????
uint8_t EC11_GetKey(void)
{
  uint8_t ret = ec11.key_flag;
  ec11.key_flag = 0;
  return ret;
}

//???????
int16_t EC11_GetCnt(void)
{
  return ec11.cnt;
}

//?????
void EC11_ClearCnt(void)
{
  ec11.cnt = 0;
  ec11.dir = 0;
}
