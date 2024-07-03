//
// Created by BxW on 2024/7/2.
//

#ifndef R2_MASTER_V3_SCREEN_H
#define R2_MASTER_V3_SCREEN_H
#include "stm32g474xx.h"
#include "main.h"
#define COMPTITION_MODE 1
#define DEBUG_MODE 2
#define DEBUG_SUCTION 3
#define DEBUG_CLAW 4
void Read_Screen_CMD(uint8_t *buffer);
void Write_Screen_CMD(uint8_t buffer);//NONE
void Unpack_Screen_CMD(const uint8_t USART_BUFFER[]);
#endif //R2_MASTER_V3_SCREEN_H
