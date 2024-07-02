//
// Created by BxW on 2024/7/2.
//

#ifndef R2_MASTER_V3_SCREEN_H
#define R2_MASTER_V3_SCREEN_H
#include "stm32g474xx.h"
#define RED 1
#define BLUE 2
#define START 3
void Read_Screen_CMD(uint8_t *buffer);
void Write_Screen_CMD(uint8_t buffer);//NONE
void Unpack_Screen_CMD(const uint8_t USART_BUFFER[]);
#endif //R2_MASTER_V3_SCREEN_H
