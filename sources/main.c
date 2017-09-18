#include "nrf.h"
#include "ble.h"
#include "nrf_sdm.h"

#include <stdlib.h>

static uint8_t buffer_in[0x80] = {0x12,0x34,0x56,0x78};
static uint8_t buffer_out[0x80] = {0x12,0x34,0x56,0x78};
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

void assertion_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  while(1){
    NVIC_SystemReset();
  };
}
static ble_gap_adv_params_t m_adv_params;   
uint32_t ble_adv_init()
{
  uint32_t ret_code;
  nrf_clock_lf_cfg_t clock;
  ble_cfg_t ble_cfg;
  uint32_t *app_ram_start = (uint32_t*)0x2000b668;
  
  clock.source = NRF_CLOCK_LF_SRC_XTAL;
  clock.rc_ctiv = 0;
  clock.rc_temp_ctiv = 0;
  clock.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
  ret_code = sd_softdevice_enable(&clock, assertion_handler);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  NVIC_EnableIRQ(SWI2_EGU2_IRQn);
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag = 1;
  ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = 1;
  ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 3;
  
  ret_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
  ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;           
  ret_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag                 = 1;
  ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = 23;
  ret_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
    
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 0;
  ret_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_gatts_cfg_attr_tab_size_t table =
  {
      .attr_tab_size = 248
  };
  ble_cfg.gatts_cfg.attr_tab_size = table;
  ret_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_gatts_cfg_service_changed_t sc =
  {
      .service_changed = 0
  };
  ble_cfg.gatts_cfg.service_changed = sc;
  ret_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  ret_code = sd_ble_enable(app_ram_start);
  
  // Adverizing init
  static uint8_t data[0x23] = "     BGM111 S";
  static uint8_t scan_data[0x23] = "  ";
  data[0] = 0x02;
  data[1] = 0x01;
  data[2] = 0x04;
  data[3] = 0x09;
  data[4] = 0x08;
  ret_code = sd_ble_gap_adv_data_set(data, 13, 0, 0);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  //
  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
  m_adv_params.p_peer_addr = 0;    // Undirected advertisement.
  m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval    = 50*20*8; //(100*1000/625)
  m_adv_params.timeout     = 0;       // Never time out.
  ret_code = sd_ble_gap_adv_start(&m_adv_params, 1);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
}


#define SCAN_INTERVAL             0x00A0                                /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000    
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist  = 0,
        .adv_dir_report = 0,
    #endif
};
#define MIN_CONNECTION_INTERVAL   (7.5*1000/1250)      /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   (30*1000/1250)       /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                     /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       (4000*1000/10000)       /**< Determines supervision time-out in units of 10 milliseconds. */

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};
uint32_t ble_central_init()
{
  uint32_t ret_code;
  nrf_clock_lf_cfg_t clock;
  ble_cfg_t ble_cfg;
  uint32_t *app_ram_start = (uint32_t*)0x2000b668;
  
  clock.source = NRF_CLOCK_LF_SRC_XTAL;
  clock.rc_ctiv = 0;
  clock.rc_temp_ctiv = 0;
  clock.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
  ret_code = sd_softdevice_enable(&clock, assertion_handler);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  NVIC_EnableIRQ(SWI2_EGU2_IRQn);
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag = 1;
  ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = 8;
  ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 3;
  
  ret_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 0;
  ble_cfg.gap_cfg.role_count_cfg.central_role_count = 8;
  ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 1;           
  ret_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag                 = 1;
  ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = 23;
  ret_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
    
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1;
  ret_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_gatts_cfg_attr_tab_size_t table =
  {
      .attr_tab_size = 1408
  };
  ble_cfg.gatts_cfg.attr_tab_size = table;
  ret_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_gatts_cfg_service_changed_t sc =
  {
      .service_changed = 0
  };
  ble_cfg.gatts_cfg.service_changed = sc;
  ret_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, *app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }
  
  ret_code = sd_ble_enable(app_ram_start);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }  
  ret_code = sd_ble_gap_scan_start(&m_scan_params);
  if (ret_code != NRF_SUCCESS)
  {
      return ret_code;
  }  
}
static uint32_t adv_report_count = 0;
static uint32_t connect = 1;
static uint32_t connected = 0;
static uint32_t conn_cnt = 0;
static uint32_t disconn_cnt = 0; 
static void ble_handler()
{
  uint32_t ret_code = 0;
  ble_evt_t * p_ble_evt;
  uint8_t evt_buffer[56];
  uint16_t evt_len = 56;
  ret_code = sd_ble_evt_get(evt_buffer, &evt_len);
  if (ret_code != NRF_SUCCESS)
  {
      return;
  }
  p_ble_evt = (ble_evt_t *)evt_buffer;
  
  switch(p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      {
        connected = 1;
        conn_cnt++;
      }
      break;
    case BLE_GAP_EVT_DISCONNECTED:
      {
        connected = 0;
        connect = 1;
        disconn_cnt++;
        sd_ble_gap_scan_start(&m_scan_params); 
      }
      break;
    case BLE_GAP_EVT_ADV_REPORT:
      {
        uint8_t conn_cfg_tag;
        ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
        ble_gap_addr_t const * p_peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
        if(connect)
        {
          sd_ble_gap_connect(p_peer_addr,&m_scan_params,&m_connection_param,1);
          connect = 0;
        }
      }
      adv_report_count++;
      break;
  }
  
}
void SWI2_EGU2_IRQHandler()
{
  ble_handler();
}
void main()
{
  //ble_adv_init();
  ble_central_init();
  static uint32_t pins = 0;
  //SCB->VTOR = 0x23000;
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