#include "nrf.h"
#include "ble.h"
#include "nrf_sdm.h"

static uint8_t buffer_in[0x4] = {0x12,0x34,0x56,0x78};
static uint8_t buffer_out[0x4] = {0x12,0x34,0x56,0x78};
void delay(uint32_t tick)
{
  volatile uint32_t temp = tick;
  while(temp)
  {
    temp--;
  }
}
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
  if(NRF_SPIS0->EVENTS_ACQUIRED)
  {
    NRF_SPIS0->RXD.PTR = (uint32_t)buffer_in;
    NRF_SPIS0->RXD.MAXCNT = sizeof(buffer_in);
    
    NRF_SPIS0->TXD.PTR = (uint32_t)buffer_out;
    NRF_SPIS0->TXD.MAXCNT = sizeof(buffer_out);
    
    NRF_SPIS0->TASKS_RELEASE = 0x1UL;
    NRF_SPIS0->EVENTS_ACQUIRED = 0x0;
  }
  if(NRF_SPIS0->EVENTS_ENDRX)
  {
    NRF_SPIS0->EVENTS_ENDRX = 0x0;
  }
  if(NRF_SPIS0->EVENTS_END)
  {
    NRF_SPIS0->EVENTS_END = 0x0;
  }
//  NRF_SPIS0->INTENSET = (SPIS_INTENSET_ACQUIRED_Set << SPIS_INTENSET_ACQUIRED_Pos) | 
//    (SPIS_INTENSET_ENDRX_Set << SPIS_INTENSET_ENDRX_Pos) |
//    (SPIS_INTENSET_END_Set << SPIS_INTENSET_ENDRX_Pos);
}
void spis_init(void)
{
//  NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 1);
  NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
  NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
  
  NRF_SPIS0->PSEL.SCK  = 11;
  NRF_SPIS0->PSEL.MISO = 13;
  NRF_SPIS0->PSEL.MOSI = 12;
  NRF_SPIS0->PSEL.CSN  = 14;
  
  NRF_SPIS0->INTENSET = (SPIS_INTENSET_ACQUIRED_Set << SPIS_INTENSET_ACQUIRED_Pos) | 
    //(SPIS_INTENSET_ENDRX_Set << SPIS_INTENSET_ENDRX_Pos) |
    (SPIS_INTENSET_END_Set << SPIS_INTENSET_END_Pos);
  
  
  NRF_SPIS0->RXD.PTR = (uint32_t)buffer_in;
  NRF_SPIS0->RXD.MAXCNT = sizeof(buffer_in);
  
  NRF_SPIS0->TXD.PTR = (uint32_t)buffer_out;
  NRF_SPIS0->TXD.MAXCNT = sizeof(buffer_out);
  
  
  NRF_SPIS0->CONFIG = (SPIS_CONFIG_CPOL_ActiveLow << SPIS_CONFIG_CPOL_Pos) | 
    (SPIS_CONFIG_CPHA_Trailing << SPIS_CONFIG_CPHA_Pos) |
    (SPIS_CONFIG_ORDER_LsbFirst << SPIS_CONFIG_ORDER_Pos);
  
  NRF_SPIS0->DEF = 0xFF;  
  NRF_SPIS0->ORC = 0xFF;  
  
  
  

  NRF_SPIS0->SHORTS = SPIS_SHORTS_END_ACQUIRE_Enabled << SPIS_SHORTS_END_ACQUIRE_Pos;
  
  NRF_SPIS0->EVENTS_END = 0;
  NRF_SPIS0->EVENTS_ACQUIRED = 0;
  NRF_SPIS0->ENABLE = 2;
  
  
  
  NRF_SPIS0->RXD.PTR = (uint32_t)buffer_in;
  NRF_SPIS0->RXD.MAXCNT = sizeof(buffer_in);
  
  NRF_SPIS0->TXD.PTR = (uint32_t)buffer_out;
  NRF_SPIS0->TXD.MAXCNT = sizeof(buffer_out);
  NRF_SPIS0->TASKS_ACQUIRE = 1;
}
void main()
{
  static uint32_t pins = 0;
  SCB->VTOR = 0x23000;
  NRF_P0->DIRCLR = 0x1F<<11;
  
  NRF_P0->PIN_CNF[11] = 0;
  NRF_P0->PIN_CNF[12] = 0;
  NRF_P0->PIN_CNF[13] = 0;
  NRF_P0->PIN_CNF[14] = 0xC;
  NRF_P0->PIN_CNF[15] = 0;
//    NRF_P0->PIN_CNF[11] = ((uint32_t)0 << GPIO_PIN_CNF_DIR_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_INPUT_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_PULL_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_DRIVE_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_SENSE_Pos);
//    NRF_P0->PIN_CNF[12] = ((uint32_t)0 << GPIO_PIN_CNF_DIR_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_INPUT_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_PULL_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_DRIVE_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_SENSE_Pos);
//    NRF_P0->PIN_CNF[13] = ((uint32_t)0 << GPIO_PIN_CNF_DIR_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_INPUT_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_PULL_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_DRIVE_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_SENSE_Pos);
//    NRF_P0->PIN_CNF[14] = ((uint32_t)0 << GPIO_PIN_CNF_DIR_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_INPUT_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_PULL_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_DRIVE_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_SENSE_Pos);
//    NRF_P0->PIN_CNF[15] = ((uint32_t)0 << GPIO_PIN_CNF_DIR_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_INPUT_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_PULL_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_DRIVE_Pos)
//                               | ((uint32_t)0 << GPIO_PIN_CNF_SENSE_Pos);
  spis_init();
  while(1){
    
    //pins = NRF_P0->IN;
    //delay(3000000);
    //NRF_P0->OUTSET = 0xF<<17;
    //delay(3000000);
    //NRF_P0->OUTCLR = 0xF<<17;
  };
}