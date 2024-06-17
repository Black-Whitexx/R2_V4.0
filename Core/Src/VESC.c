/**
  ******************************************************************************
  * @file           : VESC.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/26
  ******************************************************************************
  */
#include "VESC.h"
#include "My_Can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

extern osMessageQId SuctionSpeed_QueueHandle;

int16_t suctionSpeed;

void Vesc_SetSpeed(FDCAN_HandleTypeDef *_hcan, uint16_t motor_id, int32_t rpm)
{
    static FDCAN_TxHeaderTypeDef txHeader;
    static uint32_t mbox;
    static uint32_t tx_id;
    static union_32 temp;
    static uint8_t txData[8];

    tx_id = motor_id | (CAN_PACKET_SET_RPM) << 8;

    txHeader.Identifier=tx_id;                       //32位ID
    txHeader.IdType=FDCAN_EXTENDED_ID;                  //标准ID
    txHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    txHeader.DataLength= FDCAN_DLC_BYTES_4;             //数据长度8字节
    txHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    txHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    txHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    txHeader.MessageMarker=0x00;                        //不管捏

    // 数据先行位为倒序
    temp.data_int = rpm;
    txData[0] = temp.data_8[3];
    txData[1] = temp.data_8[2];
    txData[2] = temp.data_8[1];
    txData[3] = temp.data_8[0];

    //等一个空の邮箱呢
    while(HAL_FDCAN_GetTxFifoFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &txHeader, txData) != HAL_OK)
    {
//        printf("Error\n");
        Error_Handler();
    }
}

//void OpenVESC(void)
//{
//    suctionSpeed = 7000; /** 5065启动 **/
//    xQueueOverwrite(SuctionSpeed_QueueHandle, &suctionSpeed);
//}
//
//void StopVESC(void)
//{
//    suctionSpeed = 0; /** 5065关闭 **/
//    xQueueOverwrite(SuctionSpeed_QueueHandle, &suctionSpeed);
//}

void FanSuction(void)
{
    suctionSpeed = -2000; /** 5065关闭 **/
    xQueueOverwrite(SuctionSpeed_QueueHandle, &suctionSpeed);
}