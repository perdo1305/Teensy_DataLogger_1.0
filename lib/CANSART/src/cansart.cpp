#include "cansart.h"

extern frame11 frames11;
extern frame20 frames20;
extern frame30 frames30;
extern frame60 frames60;
extern frame121 frames121;
void cansart_init_Frames()
{
    frames11.ID = 11;
    frames20.ID = 20;
    frames30.ID = 30;
    frames60.ID = 60;
    frames121.ID = 121;
}

/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
#if !SLAVEMODE
static uint8_t BUS_Available_A = 1;
static uint8_t temp_ID_C = 0;
static uint8_t tx_verify_buffer[8];
static uint8_t BUS_Available_B = 1;
static unsigned long vTimer_TransmissionBegin = 0;
#endif



static uint8_t rx_buffer[8];
static uint8_t temp_ID_B = 0;



uint8_t startupDB = 1;

#if MCU_TYPE == C_ARDUINO
void cansart_init(HardwareSerial &serialPort, uint32_t baudrate,_vTimer vTimer)
{
    setCANSART_Driver(serialPort, baudrate, vTimer);
}
#elif MCU_TYPE == C_STM32
void cansart_init(UART_HandleTypeDef serialPort, unsigned long baudrate)
{
    setCANSART_Driver(serialPort, baudrate);
}
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#elif MCU_TYPE == C_ESP32
void cansart_init(HardwareSerial &serialPort, uint32_t baudrate, uint8_t rxPin, uint8_t txPin, _vTimer vTimer)
{
    setCANSART_Driver(serialPort, baudrate, rxPin, txPin, vTimer);
    
}
#endif

uint8_t cansart_updateDB(void *source)
{

    struct framesT dest;
    memcpy(&dest, source, sizeof(dest));

    uint8_t temp_ID_A = availableMessage(rx_buffer);
    if (temp_ID_A >= 10 && temp_ID_A <= 240)
    {
        temp_ID_B = temp_ID_A;
    }

#if !SLAVEMODE
    if (temp_ID_B == dest.ID && !BUS_Available_A)
#else
    if (temp_ID_B == dest.ID)
#endif
    {
#if !SLAVEMODE

        dest.DATA1 = rx_buffer[0];
        dest.DATA2 = rx_buffer[1];
        dest.DATA3 = rx_buffer[2];
        dest.DATA4 = rx_buffer[3];
        dest.DATA5 = rx_buffer[4];
        dest.DATA6 = rx_buffer[5];
        dest.DATA7 = rx_buffer[6];
        dest.DATA8 = rx_buffer[7];
        if (dest.ID >= 121 && dest.ID <= 140)
        {
            if (dest.DATA1 == tx_verify_buffer[0] && dest.DATA2 == tx_verify_buffer[1] && dest.DATA3 == tx_verify_buffer[2] && dest.DATA4 == tx_verify_buffer[3] && dest.DATA5 == tx_verify_buffer[4] && dest.DATA6 == tx_verify_buffer[5] && dest.DATA7 == tx_verify_buffer[6] && dest.DATA8 == 0)
            {
                temp_ID_C = 0;

                BUS_Available_B = 1;
            }
        }
        memcpy(source, &dest, sizeof(dest));
        BUS_Available_A = 1;
        temp_ID_B = 0;
        return 1;
#else
        if (temp_ID_B <= 120)
        {

            uint8_t temp_tx_buffer[8];
            temp_tx_buffer[0] = dest.DATA1;
            temp_tx_buffer[1] = dest.DATA2;
            temp_tx_buffer[2] = dest.DATA3;
            temp_tx_buffer[3] = dest.DATA4;
            temp_tx_buffer[4] = dest.DATA5;
            temp_tx_buffer[5] = dest.DATA6;
            temp_tx_buffer[6] = dest.DATA7;
            temp_tx_buffer[7] = dest.DATA8;

            transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
            temp_ID_B = 0;
        }
        else
        {
            if (!startupDB)
            {
                if (rx_buffer[7] == 1)
                {
                    dest.DATA1 = rx_buffer[0];
                    dest.DATA2 = rx_buffer[1];
                    dest.DATA3 = rx_buffer[2];
                    dest.DATA4 = rx_buffer[3];
                    dest.DATA5 = rx_buffer[4];
                    dest.DATA6 = rx_buffer[5];
                    dest.DATA7 = rx_buffer[6];
                    dest.DATA8 = 0;
                    rx_buffer[7] = 0;
                    memcpy(source, &dest, sizeof(dest));
                    transmitMessage(dest.ID, rx_buffer, dest.LENGHT);
                }
                else
                {
                    uint8_t temp_tx_buffer[8];
                    temp_tx_buffer[0] = dest.DATA1;
                    temp_tx_buffer[1] = dest.DATA2;
                    temp_tx_buffer[2] = dest.DATA3;
                    temp_tx_buffer[3] = dest.DATA4;
                    temp_tx_buffer[4] = dest.DATA5;
                    temp_tx_buffer[5] = dest.DATA6;
                    temp_tx_buffer[6] = dest.DATA7;
                    temp_tx_buffer[7] = dest.DATA8;
                    transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
                }
            }
            else
            {
                uint8_t temp_tx_buffer[8];
                temp_tx_buffer[0] = dest.DATA1;
                temp_tx_buffer[1] = dest.DATA2;
                temp_tx_buffer[2] = dest.DATA3;
                temp_tx_buffer[3] = dest.DATA4;
                temp_tx_buffer[4] = dest.DATA5;
                temp_tx_buffer[5] = dest.DATA6;
                temp_tx_buffer[6] = dest.DATA7;
                temp_tx_buffer[7] = dest.DATA8;
                transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
                startupDB = 0;
            }
            temp_ID_B = 0;
        }
        return 1;
#endif
    }
#if !SLAVEMODE
    else if (BUS_Available_A)
    {

        if (dest.ID >= 10 && dest.ID <= 120)
        {

            uint8_t tx_buffer[8] = {};

            transmitMessage(dest.ID, tx_buffer, dest.LENGHT);
            BUS_Available_A = 0;
            vTimer_TransmissionBegin = vTimer();
            return 0;
        }
        else if (dest.ID >= 121 && dest.ID <= 240)
        {

            tx_verify_buffer[0] = dest.DATA1;
            tx_verify_buffer[1] = dest.DATA2;
            tx_verify_buffer[2] = dest.DATA3;
            tx_verify_buffer[3] = dest.DATA4;
            tx_verify_buffer[4] = dest.DATA5;
            tx_verify_buffer[5] = dest.DATA6;
            tx_verify_buffer[6] = dest.DATA7;
            tx_verify_buffer[7] = 1;

            transmitMessage(dest.ID, tx_verify_buffer, 8);
            
            BUS_Available_A = 0;
vTimer_TransmissionBegin = vTimer();
            return 0;
        }
        return 2;
    }
       if (vTimer() - vTimer_TransmissionBegin > MASTER_TIMEOUT_MS)
    {
        BUS_Available_A = 1;
        return 3;
    }
#endif
 
    return 4;
}
/*
uint8_t cansart_write_slave_DB(void *source)
{
    struct framesT dest;
    memcpy(&dest, source, sizeof(struct framesT));
    if (BUS_Available_B)
    {
        if (dest.ID >= 121 && dest.ID <= 240)
        {

            tx_verify_buffer[0] = dest.DATA1;
            tx_verify_buffer[1] = dest.DATA2;
            tx_verify_buffer[2] = dest.DATA3;
            tx_verify_buffer[3] = dest.DATA4;
            tx_verify_buffer[4] = dest.DATA5;
            tx_verify_buffer[5] = dest.DATA6;
            tx_verify_buffer[6] = dest.DATA7;
            tx_verify_buffer[7] = 1;

            transmitMessage(dest.ID, tx_verify_buffer, 8);
            BUS_Available_B = 0;
            return 1;
        }
        return 2;
    }
    else if (!BUS_Available_B)
    {
        return 0;
    }
    return 3;
}
*/