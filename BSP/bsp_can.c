#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"
#include "motor.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "bsp_can.h"
volatile Fdcan1Debug_s g_fdcan1_debug;
volatile Fdcan2Debug_s g_fdcan2_debug;

FDCAN_RxHeaderTypeDef RxHeader1;  
uint8_t g_Can1RxData[64];
//声明SuperCap的接收数据结构体
SuperCap_rx_t SuperCap_rx;

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];
//extern CTOM_message_t ctom_message;
extern DmMeasure_s CAN_DM_MEASURE[DM_NUM];
extern DjiMotorMeasure_t CAN_DJI_MEASURE[5];
//void C_communication_M(CTOM_message_t *ctom_mesasge,uint8_t *rx_data)
//{
//   ctom_mesasge->gimbal_yaw_6020 = (int16_t)(rx_data[0] << 8 | rx_data[1]) / 1000.0f;
//	 ctom_mesasge->gimbal_motor_offline=rx_data[2];
//	 ctom_mesasge->aim_live=rx_data[3];
//	ctom_mesasge->aim_shoot=rx_data[4];
//	
//}


void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 在这里设置不滤除所有ID
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 

  /* Start the FDCAN module */
  if (hfdcan1.Init.FrameFormat == FDCAN_FRAME_FD_BRS)
  {
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
          hfdcan1.Init.DataPrescaler * (1U + hfdcan1.Init.DataTimeSeg1), 0) != HAL_OK ||
        HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
      Error_Handler();
    }
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  g_fdcan1_debug.started = 1;
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  g_fdcan2_debug.started = 1U;
}

void FDCAN3_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;//只接受范围内的报文
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x400;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
}



uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader = {0};
  HAL_StatusTypeDef status;

  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = len; /* Existing callers use DLC values 0..8. */
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.FDFormat = (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
                      ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
  TxHeader.BitRateSwitch = (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS)
                           ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
  if (hfdcan == &hfdcan1)
  {
    g_fdcan1_debug.last_tx_id = id;
    if (status == HAL_OK)
      g_fdcan1_debug.tx_enqueued++;
    else
      g_fdcan1_debug.tx_failed++;
    g_fdcan1_debug.tx_free = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    g_fdcan1_debug.hal_error = HAL_FDCAN_GetError(hfdcan);
  }
  if (hfdcan == &hfdcan2)
  {
    g_fdcan2_debug.last_tx_id = id;
    if (status == HAL_OK)
      g_fdcan2_debug.tx_enqueued++;
    else
      g_fdcan2_debug.tx_failed++;
  }
  return (status == HAL_OK) ? 0 : 1;
}

/* FDCAN1: drain FIFO0 and count raw frames before checking feedback IDs. */
uint8_t rx_data1[64] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef header;
  uint32_t index;

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, rx_data1) != HAL_OK)
      break;

    rec_id1 = (uint16_t)header.Identifier;
    g_fdcan1_debug.rx_count++;
    g_fdcan1_debug.last_rx_id = header.Identifier;
    g_fdcan1_debug.last_rx_dlc = header.DataLength;
    g_fdcan1_debug.last_rx_fd = header.FDFormat;
    g_fdcan1_debug.last_rx_brs = header.BitRateSwitch;

    /* Master IDs 0x11..0x14 map to motor CAN IDs 1..4; feedback is 8 bytes. */
    if (header.IdType == FDCAN_STANDARD_ID &&
        header.RxFrameType == FDCAN_DATA_FRAME &&
        header.DataLength == FDCAN_DLC_BYTES_8 &&
        header.Identifier >= 0x11U && header.Identifier <= 0x14U)
    {
      index = header.Identifier - 0x11U;
      DmFdbData(&CAN_DM_MEASURE[index], rx_data1);
      g_fdcan1_debug.parsed_count++;
    }
    else
    {
      g_fdcan1_debug.rejected_count++;
    }
  }
}

/* PSR reads reset LEC/DLEC reporting; retain the last real error for Watch. */
void FDCAN1_UpdateDebug(void)
{
  static uint32_t last_poll;
  FDCAN_ProtocolStatusTypeDef protocol;
  FDCAN_ErrorCountersTypeDef counters;
  uint32_t now = HAL_GetTick();

  g_fdcan1_debug.loop_tick = now;
  if ((uint32_t)(now - last_poll) < 100U)
    return;
  last_poll = now;

  g_fdcan1_debug.kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  g_fdcan1_debug.nbtp = hfdcan1.Instance->NBTP;
  g_fdcan1_debug.dbtp = hfdcan1.Instance->DBTP;
  g_fdcan1_debug.cccr = hfdcan1.Instance->CCCR;
  g_fdcan1_debug.tx_pending_mask = hfdcan1.Instance->TXBRP;
  g_fdcan1_debug.tx_occurred_mask |= hfdcan1.Instance->TXBTO;
  g_fdcan1_debug.rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) == HAL_OK)
  {
    g_fdcan1_debug.bus_off = protocol.BusOff;
    if (protocol.LastErrorCode > 0U && protocol.LastErrorCode < 7U)
      g_fdcan1_debug.last_lec = protocol.LastErrorCode;
    if (protocol.DataLastErrorCode > 0U && protocol.DataLastErrorCode < 7U)
      g_fdcan1_debug.last_dlec = protocol.DataLastErrorCode;
  }
  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &counters) == HAL_OK)
  {
    g_fdcan1_debug.tx_error_count = counters.TxErrorCnt;
    g_fdcan1_debug.rx_error_count = counters.RxErrorCnt;
  }
  g_fdcan1_debug.hal_error = HAL_FDCAN_GetError(&hfdcan1);
  g_fdcan1_debug.tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
}

/* Poll diagnostics at 10 Hz; preserve existing CAN2 receive/control logic. */
void FDCAN2_UpdateDebug(void)
{
  static uint32_t last_poll_tick;
  uint32_t now = HAL_GetTick();
  uint32_t left_tick;
  uint32_t right_tick;
  FDCAN_ProtocolStatusTypeDef protocol;
  FDCAN_ErrorCountersTypeDef counters;
  if ((uint32_t)(now - last_poll_tick) < 100U)
    return;
  last_poll_tick = now;

  g_fdcan2_debug.kernel_clock_hz =
      HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  g_fdcan2_debug.nominal_bitrate = g_fdcan2_debug.kernel_clock_hz /
      (hfdcan2.Init.NominalPrescaler *
       (1U + hfdcan2.Init.NominalTimeSeg1 + hfdcan2.Init.NominalTimeSeg2));
  g_fdcan2_debug.cccr = hfdcan2.Instance->CCCR;
  g_fdcan2_debug.ir = hfdcan2.Instance->IR;
  g_fdcan2_debug.ie = hfdcan2.Instance->IE;
  g_fdcan2_debug.ils = hfdcan2.Instance->ILS;
  g_fdcan2_debug.ile = hfdcan2.Instance->ILE;
  g_fdcan2_debug.rx_fifo1_fill =
      HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1);
  g_fdcan2_debug.tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
  g_fdcan2_debug.tx_pending_mask = hfdcan2.Instance->TXBRP;
  g_fdcan2_debug.tx_occurred_mask |= hfdcan2.Instance->TXBTO;
  g_fdcan2_debug.hal_error = HAL_FDCAN_GetError(&hfdcan2);

  /* Read timestamps before sampling now, avoiding wrap if an ISR updates them. */
  left_tick = CAN_DJI_MEASURE[0].last_fdb_time;
  right_tick = CAN_DJI_MEASURE[1].last_fdb_time;
  now = HAL_GetTick();
  g_fdcan2_debug.left_feedback_age_ms = (uint32_t)(now - left_tick);
  g_fdcan2_debug.right_feedback_age_ms = (uint32_t)(now - right_tick);
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol) == HAL_OK)
  {
    g_fdcan2_debug.bus_off = protocol.BusOff;
    if (protocol.LastErrorCode > 0U && protocol.LastErrorCode < 7U)
      g_fdcan2_debug.last_lec = protocol.LastErrorCode;
  }
  if (HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters) == HAL_OK)
  {
    g_fdcan2_debug.tx_error_count = counters.TxErrorCnt;
    g_fdcan2_debug.rx_error_count = counters.RxErrorCnt;
  }
}

//CAN通信收发ID
#define        CAN_SUPERCAP_RX_ID 0x051
#define        CAN_SUPERCAP_TX_ID 0x061






//SuperCap的结构体接受函数                
void get_SuperCap_measure(SuperCap_rx_t* ptr, uint8_t *data)
{
        ptr->errorCode = data[0];
        memcpy(&ptr->chassisPower,&data[1], sizeof(float));
        ptr->chassisPowerLimit = (uint16_t)(data[6]<<8 | data[7]);
        ptr->capEnergy = data[7];
}




/*                 fdcan2接收                         */
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
SuperCap_rx_t SuperCap_rx;
void fdcan2_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
    g_fdcan2_debug.rx_callback_count++;
	
	if(HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO1,&pRxHeader, rx_data2)==HAL_OK)
	{
		rec_id2 = pRxHeader.Identifier;
        g_fdcan2_debug.rx_count++;
        g_fdcan2_debug.last_rx_id = pRxHeader.Identifier;
        g_fdcan2_debug.last_rx_dlc = pRxHeader.DataLength;
        g_fdcan2_debug.last_rx_tick = HAL_GetTick();
        if (pRxHeader.DataLength > FDCAN_DLC_BYTES_8 ||
            (rec_id2 != 0x201U && rec_id2 != 0x202U))
          g_fdcan2_debug.rejected_count++;

	if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) //经典CAN里面只有8个字节   大疆3508电机  左0x0202   右0x0201
	{
			if(rec_id2==0x0202)
		 {
			 DjiFdbData(&CAN_DJI_MEASURE[1], rx_data2);
         g_fdcan2_debug.rx_id202_count++;
         g_fdcan2_debug.parsed_count++;//左轮毂
		 }
		 if(rec_id2==0x0201)
		 {
  		 DjiFdbData(&CAN_DJI_MEASURE[0], rx_data2);
         g_fdcan2_debug.rx_id201_count++;
         g_fdcan2_debug.parsed_count++;
		 }

//		 if(rec_id2==CAN_SUPERCAP_RX_ID)
//		 {
//			 get_SuperCap_measure(&SuperCap_rx,rx_data2);//超电
//		 }
	
	
	}
}
else
{
    g_fdcan2_debug.rx_read_failed++;
}
}
/*                 fdcan3接收                         */
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(&hfdcan3,FDCAN_RX_FIFO0,&pRxHeader, rx_data3)==HAL_OK)
	{
		rec_id3 = pRxHeader.Identifier;

	if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) //经典CAN里面只有8个字节   大疆3508电机  左0x0202   右0x0201
	{
//      if(rec_id3==0x666)//C板下发数据
//     {
//       C_communication_M(&ctom_message,rx_data3);
//     }

	}
	}
}

/* 修正后的RX FIFO0回调函数 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* 首先检查是否真的是"新消息"中断，而不是溢出等其他中断 */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        if (hfdcan == &hfdcan1) {
            fdcan1_rx_callback(); // CAN1使用FIFO0
        }
        if (hfdcan == &hfdcan3) {
            fdcan3_rx_callback(); // CAN3使用FIFO0
        }
        // CAN2配置为使用FIFO1，所以不应在FIFO0的回调中处理
    }
}

/* 修正后的RX FIFO1回调函数 - 注意参数名改为RxFifo1ITs */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) // 注意：参数名已修正！
{
    /* 检查FIFO1的新消息中断 */
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0) {
        if (hfdcan == &hfdcan2) {
            fdcan2_rx_callback(); // CAN2使用FIFO1
        }
        // CAN1和CAN3不使用FIFO1，所以不在这里处理
    }  
}




