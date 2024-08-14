#ifndef CANSART_DB_H
#define CANSART_DB_H

#include "stdint.h"

/**
 * @brief Define the MCU type
 * 
 * @param C_ARDUINO
 * @param C_STM32 
 * @param C_PIC32 
 * @param C_RENESAS
 * @param C_ESP32
 */
#define MCU_TYPE C_ARDUINO

/**
 * @brief Define the mode of the device - always need a master and a slave on the network
 * 
 * @param MASTERMODE : Works as data requester and data provider
 * @param SLAVEMODE  : Works as data provider
 */
#define SLAVEMODE 1

struct frame11
{
    uint8_t ID ;
    uint8_t DATA1 ;
    uint8_t DATA2;
    uint8_t DATA3 ;
    uint8_t DATA4 ;
    uint8_t DATA5 ;
    uint8_t DATA6 ;
    uint8_t DATA7 ;
    uint8_t DATA8 ;
    uint8_t LENGHT ;
};

struct frame20
{
    uint8_t ID ;
    uint8_t DATA1 ;
    uint8_t DATA2;
    uint8_t DATA3 ;
    uint8_t DATA4 ;
    uint8_t DATA5 ;
    uint8_t DATA6 ;
    uint8_t DATA7 ;
    uint8_t DATA8 ;
    uint8_t LENGHT ;
};

struct frame30
{
    uint8_t ID ;
    uint8_t DATA1 ;
    uint8_t DATA2;
    uint8_t DATA3 ;
    uint8_t DATA4 ;
    uint8_t DATA5 ;
    uint8_t DATA6 ;
    uint8_t DATA7 ;
    uint8_t DATA8 ;
    uint8_t LENGHT ;
};

struct frame60
{
    uint8_t ID ;
    uint8_t DATA1 ;
    uint8_t DATA2;
    uint8_t DATA3 ;
    uint8_t DATA4 ;
    uint8_t DATA5 ;
    uint8_t DATA6 ;
    uint8_t DATA7 ;
    uint8_t DATA8 ;
    uint8_t LENGHT ;
};

struct frame121
{
    uint8_t ID ;
    uint8_t DATA1 ;
    uint8_t DATA2;
    uint8_t DATA3 ;
    uint8_t DATA4 ;
    uint8_t DATA5 ;
    uint8_t DATA6 ;
    uint8_t DATA7 ;
    uint8_t DATA8 ;
    uint8_t LENGHT ;
};


#endif