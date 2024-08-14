#ifndef CANSARTP_H
#define CANSARTP_H

#include "cansart_db.h"
#include "cansartP.h"
#include "string.h"
#include "stdlib.h"
#include "driver.h"

#if (MCU_TYPE == C_ARDUINO || MCU_TYPE == C_ESP32)

#include <Arduino.h>

#elif MCU_TYPE == C_STM32
// To include
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#endif

uint8_t transmitMessage(uint8_t ID, uint8_t *txmessageBuffer, uint8_t messageLength);
void tx_checksum_calculator();
uint8_t availableMessage(uint8_t *rxmessageBuffer);
uint8_t rx_checksum_calculator();
uint8_t convertData(char *receivedData);
uint8_t receiveUSART(char *buffer);

#endif
