#include "driver.h"

_vTimer C_vTimer = nullptr;

#if MCU_TYPE == C_ARDUINO
HardwareSerial *Lusart;
void setCANSART_Driver(HardwareSerial &usart, unsigned long baudrate,_vTimer vTimer)
{
  Lusart = &usart;
  Lusart->begin(baudrate);
  C_vTimer = vTimer;
}
#elif MCU_TYPE == STM32

UART_HandleTypeDef Lusart;

uint8_t mainBuffer[49];

UART_HandleTypeDef Cusart;

volatile uint8_t uartDone = 0;

void setCANSART_Driver(UART_HandleTypeDef usart, unsigned long baudrate) {
	Lusart = usart;
	//HAL_UARTEx_ReceiveToIdle_DMA(&Lusart, mainBuffer, 49);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, mainBuffer, 49);
}

#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#elif MCU_TYPE == C_ESP32
HardwareSerial *Lusart;
void setCANSART_Driver(HardwareSerial &usart, unsigned long baudrate, uint8_t rxPin, uint8_t txPin,_vTimer vTimer)
{
  Lusart = &usart;
  Lusart->begin(baudrate, SERIAL_8N1, rxPin, txPin);
  C_vTimer = vTimer;
}
#endif

void sendData(uint16_t sendByte) {
#if (MCU_TYPE == C_ARDUINO || MCU_TYPE == C_ESP32)
  Lusart->print(sendByte);
  Lusart->print('\0');
#elif MCU_TYPE == C_STM32
	//HAL_UART_Transmit(&Lusart, (uint8_t *)&sendByte, 1, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&Lusart, (uint8_t *)'\0', 1, HAL_MAX_DELAY);
	// Buffer to hold the string representation of the number
	char buffer[6]; // Enough to hold "65535" and the null terminator

	// Convert the integer to a string
	sprintf(buffer, "%u", sendByte);
	uint8_t len = strlen(buffer);
	uint8_t len_aux = len - 1;
	// Transmit the string
	char myChar = '\0';
	for(int i = len_aux; i >= 0; i--){
		myChar = buffer[(len_aux - i)];
		HAL_UART_Transmit(&huart1, (uint8_t*)&myChar, 1,HAL_MAX_DELAY);
	}
	myChar = '\0';

	HAL_UART_Transmit(&huart1, (uint8_t*) &myChar, 1, HAL_MAX_DELAY);

#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#endif
}

void sendFinishData() {
#if (MCU_TYPE == C_ARDUINO || MCU_TYPE == C_ESP32)
  Lusart->print('\n');
#elif MCU_TYPE == C_STM32
	// HAL_UART_Transmit(&Lusart, (uint8_t *)'\n', 1, HAL_MAX_DELAY);
  char myChar = '\n';
	HAL_UART_Transmit(&huart1, (uint8_t*) &myChar, 1, HAL_MAX_DELAY);
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#endif
}

int newData() {
#if (MCU_TYPE == C_ARDUINO || MCU_TYPE == C_ESP32)
  return Lusart->available();
#elif MCU_TYPE == C_STM32
	if (uartDone) {
		uartDone = 0;

		return 1;
	} else {

		return 0;
	}
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#endif
}

uint8_t getData(uint8_t *pData) {
#if (MCU_TYPE == C_ARDUINO || MCU_TYPE == C_ESP32)
  return Lusart->read();
#elif MCU_TYPE == C_STM32
	memcpy(pData, mainBuffer, sizeof mainBuffer);
	memset(mainBuffer, 0, sizeof mainBuffer);
	// HAL_UARTEx_ReceiveToIdle_DMA(&Lusart, mainBuffer, 49);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, mainBuffer, 49);
	return 1;
#elif MCU_TYPE == C_PIC32
// To include
#elif MCU_TYPE == C_RENESAS
// To include
#endif
}

#if MCU_TYPE == C_STM32
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	uartDone = 1;
}
#endif

unsigned long vTimer()
{
    if (C_vTimer != nullptr)
    {
        return C_vTimer();  //return time since controller started in ms(milliseconds)
    }
    else
    {
        return 0;
    }
}
