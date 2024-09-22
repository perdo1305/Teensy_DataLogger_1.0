
#include <stdint.h>

typedef struct {
    uint16_t ERPM;
    uint16_t Duty;
    uint16_t InputVoltage;

    uint16_t ACCurrent;
    uint16_t DCCurrent;

    uint16_t TempController;
    uint16_t TempMotor;
    uint8_t FaultCode;

    uint32_t FOC_id;
    uint32_t FOC_iq;

    uint8_t Throttle;
    uint8_t Brake;
    uint8_t Digital_input_1;
    uint8_t Digital_input_2;
    uint8_t Digital_input_3;
    uint8_t Digital_input_4;
    uint8_t Digital_output_1;
    uint8_t Digital_output_2;
    uint8_t Digital_output_3;
    uint8_t Digital_output_4;
    uint8_t Drive_enable;
    uint8_t Capacitor_temp_limit;
    uint8_t DC_current_limit;
    uint8_t Drive_enable_limit;
} HV500_t;