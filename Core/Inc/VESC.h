/**
  ******************************************************************************
  * @file           : VESC.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/26
  ******************************************************************************
  */

#ifndef R2_MECHANISM_V3_VESC_H
#define R2_MECHANISM_V3_VESC_H
#include "main.h"

#define VESC_ID 0X79

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5,
    CAN_PACKET_POLL_TS5700N8501_STATUS,
    CAN_PACKET_CONF_BATTERY_CUT,
    CAN_PACKET_CONF_STORE_BATTERY_CUT,
    CAN_PACKET_SHUTDOWN,
    CAN_PACKET_IO_BOARD_ADC_1_TO_4,
    CAN_PACKET_IO_BOARD_ADC_5_TO_8,
    CAN_PACKET_IO_BOARD_ADC_9_TO_12,
    CAN_PACKET_IO_BOARD_DIGITAL_IN,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
    CAN_PACKET_BMS_V_TOT,
    CAN_PACKET_BMS_I,
    CAN_PACKET_BMS_AH_WH,
    CAN_PACKET_BMS_V_CELL,
    CAN_PACKET_BMS_BAL,
    CAN_PACKET_BMS_TEMPS,
    CAN_PACKET_BMS_HUM,
    CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,
    CAN_PACKET_PSW_STAT,
    CAN_PACKET_PSW_SWITCH,
    CAN_PACKET_BMS_HW_DATA_1,
    CAN_PACKET_BMS_HW_DATA_2,
    CAN_PACKET_BMS_HW_DATA_3,
    CAN_PACKET_BMS_HW_DATA_4,
    CAN_PACKET_BMS_HW_DATA_5,
    CAN_PACKET_BMS_AH_WH_CHG_TOTAL,
    CAN_PACKET_BMS_AH_WH_DIS_TOTAL,
    CAN_PACKET_UPDATE_PID_POS_OFFSET,
    CAN_PACKET_POLL_ROTOR_POS,
    CAN_PACKET_BMS_BOOT,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
}CAN_ID_Packet;

void Vesc_SetSpeed(FDCAN_HandleTypeDef *_hcan, uint16_t motor_id, int32_t rpm);
//void OpenVESC(void);
//void StopVESC(void);
void FanSuction(void);

#endif //R2_MECHANISM_V3_VESC_H
