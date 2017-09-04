#include "nrf.h"
#include "ble.h"
#include "nrf_sdm.h"

void delay(uint32_t tick)
{
  volatile uint32_t temp = tick;
  while(temp)
  {
    temp--;
  }
}

void main()
{
  NRF_P0->DIRSET = 0xF<<17;
  while(1){
    delay(3000000);
    NRF_P0->OUTSET = 0xF<<17;
    delay(3000000);
    NRF_P0->OUTCLR = 0xF<<17;
  };
}