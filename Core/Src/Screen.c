//
// Created by BxW on 2024/7/2.
//
#include "Screen.h"
uint8_t Screen_CMD;
void Read_Screen_CMD(uint8_t *buffer){
    *buffer = Screen_CMD;
}
void Write_Screen_CMD(uint8_t buffer){
    //NONE
}

void Unpack_Screen_CMD(const uint8_t USART_BUFFER[]){
    if(USART_BUFFER[0] == 0xAC && USART_BUFFER[2] == 0xFF){
        if(USART_BUFFER[1] == 0x01){
            Screen_CMD = RED;
        }
        else if(USART_BUFFER[1] == 0x02){
            Screen_CMD = BLUE;
        }
    }
    if(USART_BUFFER[0] == 0xAE && USART_BUFFER[1] == 0xFF){
        Screen_CMD = START;
    }

}