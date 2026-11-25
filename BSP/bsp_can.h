#ifndef _BSP_CAN_H
#define _BSP_CAN_H


#include "main.h"
//#include "CAN_receive.h"

typedef FDCAN_HandleTypeDef hcan_t;

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;
//超级电容接收数据信息
typedef struct
{
    uint8_t errorCode;//错误代码
    float chassisPower;//底盘实时功率
    uint16_t chassisPowerLimit;//底盘功率限制，超出会进行功率补偿
    uint8_t capEnergy;//值域为：0-255 255是满电
}SuperCap_rx_t;



/* Watch this structure in Keil. Enqueued does not mean acknowledged on CAN. */
typedef struct
{
    uint32_t started;
    uint32_t tx_enqueued;
    uint32_t tx_failed;
    uint32_t tx_free;
    uint32_t last_tx_id;
    uint32_t rx_count;
    uint32_t parsed_count;
    uint32_t rejected_count;
    uint32_t last_rx_id;
    uint32_t last_rx_dlc;
    uint32_t last_rx_fd;
    uint32_t last_rx_brs;
    uint32_t hal_error;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t bus_off;
    uint32_t last_lec;
    uint32_t last_dlec;
    uint32_t loop_tick;
    uint32_t kernel_clock_hz; /* RCC-derived; assumes correct HSE_VALUE. */
    uint32_t nbtp;
    uint32_t dbtp;
    uint32_t cccr;
    uint32_t tx_pending_mask;
    uint32_t tx_occurred_mask; /* Sticky observation of hardware TXBTO bits. */
    uint32_t rx_fifo0_fill;
} Fdcan1Debug_s;

extern volatile Fdcan1Debug_s g_fdcan1_debug;
void FDCAN1_UpdateDebug(void);

/* CAN2 diagnostics. Counters are cumulative; last_lec retains the last error.
 * tx_enqueued counts queue acceptance, not delivery/acknowledgment.
 * Clock/bitrate are RCC-derived and assume HSE_VALUE matches the board.
 */
typedef struct
{
    uint32_t started;
    uint32_t rx_callback_count;
    uint32_t rx_count;
    uint32_t parsed_count;
    uint32_t rejected_count;
    uint32_t rx_id201_count;
    uint32_t rx_id202_count;
    uint32_t rx_read_failed;
    uint32_t last_rx_id;
    uint32_t last_rx_dlc;
    uint32_t last_rx_tick;
    uint32_t left_feedback_age_ms;
    uint32_t right_feedback_age_ms;
    uint32_t tx_enqueued;
    uint32_t tx_failed;
    uint32_t last_tx_id;
    uint32_t tx_free;
    uint32_t tx_pending_mask;
    uint32_t tx_occurred_mask;
    uint32_t rx_fifo1_fill;
    uint32_t hal_error;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t bus_off;
    uint32_t last_lec;
    uint32_t kernel_clock_hz;
    uint32_t nominal_bitrate;
    uint32_t cccr;
    uint32_t ir;
    uint32_t ie;
    uint32_t ils;
    uint32_t ile;
} Fdcan2Debug_s;

extern volatile Fdcan2Debug_s g_fdcan2_debug;
void FDCAN2_UpdateDebug(void);

void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);



#endif
