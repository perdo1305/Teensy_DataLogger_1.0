#ifndef DRIVER_H
#define DRIVER_H

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "cansart_db.h"

#define C_ARDUINO 0
#define C_STM32 1
#define C_PIC32 2
#define C_RENESAS 3
#define C_ESP32 4

typedef unsigned long (*_vTimer)();

#if (MCU_TYPE == C_ARDUINO)

#include "HardwareSerial.h"
void setCANSART_Driver(HardwareSerial &usart, unsigned long baudrate,_vTimer vTimer);

#elif MCU_TYPE == C_STM32
#include "stm32f1xx_hal.h"
#include "usart.h"
void setCANSART_Driver(UART_HandleTypeDef usart,unsigned long baudrate);
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#elif MCU_TYPE == C_ESP32
#include "HardwareSerial.h"
void setCANSART_Driver(HardwareSerial &usart, unsigned long baudrate, uint8_t rxPin, uint8_t txPin,_vTimer vTimer);
#endif




void sendData(uint16_t sendByte);
void sendFinishData();

int newData();
uint8_t getData(uint8_t *pData);

unsigned long vTimer();

#endif
