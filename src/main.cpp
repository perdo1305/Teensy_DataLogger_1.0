/**
 * @file main.cpp
 * @brief CAN voltage bridge and battery charger handcart controller
 * @author Pedro Ferreira
 */

#include <FlexCAN_T4.h>

// Battery Charger State Machine enum (must be declared before function prototypes)
enum ChargerState {
    INITIAL,
    IDLE,
    CHARGING,
    SHUTDOWN
};

// Function prototypes
void updateChargerState();
void sendBMSCommand();
void printState(ChargerState state);

// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t rxmsg;  // Struct to hold received CAN message
CAN_message_t txmsg;  // Struct to hold sent CAN message

// Global variable to store received voltage
int32_t received_voltage_mv = 0;             // Voltage in mV from IVT
bool voltage_received = false;               // Flag to indicate if we have received voltage data
unsigned long last_voltage_time = 0;         // Last time voltage was received
const unsigned long VOLTAGE_TIMEOUT = 5000;  // 5 seconds timeout in ms

// Voltage transmission delay variables
int32_t pending_voltage_mv = 0;          // Voltage waiting to be sent
bool voltage_pending = false;            // Flag for voltage pending transmission
unsigned long voltage_receive_time = 0;  // Time when voltage was received
const unsigned long VOLTAGE_DELAY = 50;  // 50ms delay before sending

// LED timing variables
unsigned long led33_start = 0;         // Start time for GPIO 33 LED
unsigned long led36_start = 0;         // Start time for GPIO 36 LED
bool led33_active = false;             // GPIO 33 LED state
bool led36_active = false;             // GPIO 36 LED state
const unsigned long LED_DURATION = 5;  // LED on duration in ms

// Battery Charger State Machine variables
ChargerState current_state = INITIAL;
const int SWITCH_PIN = 25;    // Digital input for user switch
const int SHUTDOWN_PIN = 24;  // Digital input for shutdown circuit
bool switch_state = false;
bool shutdown_state = false;
unsigned long last_state_send = 0;
const unsigned long STATE_SEND_INTERVAL = 100;  // Send state every 100ms

void setup() {
    Serial.begin(115200);

    // Initialize CAN bus
    can1.begin();
    can1.setBaudRate(250000);

    // Initialize LED pins
    pinMode(33, OUTPUT);  // LED for received messages
    pinMode(34, OUTPUT);  // LED for shutdown state
    pinMode(35, OUTPUT);  // LED for switch/ignition state
    pinMode(36, OUTPUT);  // LED for sent messages
    digitalWrite(33, LOW);
    digitalWrite(34, LOW);
    digitalWrite(35, LOW);
    digitalWrite(36, LOW);

    // Initialize charger state machine pins
    pinMode(SWITCH_PIN, INPUT);    // Switch input with pullup
    pinMode(SHUTDOWN_PIN, INPUT);  // Shutdown input with pullup

    Serial.println("CAN Bus initialized");
    Serial.println("Battery Charger Handcart Ready");
    Serial.println("Listening for CAN messages...");
}

void loop() {
    // Read digital inputs
    switch_state = digitalRead(SWITCH_PIN);      // Direct read (HIGH = switch on)
    shutdown_state = digitalRead(SHUTDOWN_PIN);  // Direct read (HIGH = circuit closed, LOW = shutdown)

    // Update LEDs to reflect input states
    digitalWrite(34, shutdown_state ? HIGH : LOW);  // GPIO 34 shows shutdown state
    digitalWrite(35, switch_state ? HIGH : LOW);    // GPIO 35 shows switch/ignition state

    // State machine logic
    updateChargerState();

    // Handle LED timing (non-blocking)
    unsigned long currentTime = millis();

    // Handle GPIO 33 LED
    if (led33_active && (currentTime - led33_start >= LED_DURATION)) {
        digitalWrite(33, LOW);
        led33_active = false;
    }

    // Handle GPIO 36 LED
    if (led36_active && (currentTime - led36_start >= LED_DURATION)) {
        digitalWrite(36, LOW);
        led36_active = false;
    }

    // Send HV500 voltage data periodically
    static unsigned long lastSend = 0;

    if (millis() - lastSend > 200) {  // Send every 200ms
        lastSend = millis();

        // Check if we have a pending voltage to send (50ms delay)
        if (voltage_pending && (millis() - voltage_receive_time >= VOLTAGE_DELAY)) {
            // Use the pending voltage for transmission
            received_voltage_mv = pending_voltage_mv;
            voltage_received = true;
            voltage_pending = false;  // Clear pending flag
        }

        // Check for voltage timeout
        if (voltage_received && (millis() - last_voltage_time > VOLTAGE_TIMEOUT)) {
            voltage_received = false;
            received_voltage_mv = 0;
            Serial.println("Voltage timeout - sending zero");
        }

        // Prepare HV500_ERPM_DUTY_VOLTAGE message (ID: 0x14 = 20 decimal)
        txmsg.id = 0x14;
        txmsg.len = 8;

        // Clear buffer
        memset(txmsg.buf, 0, 8);

        // Encode Actual_ERPM: 7|32@0- (bit 7, 32 bits, little endian, signed) - SET TO 0
        txmsg.buf[0] = 0x00;
        txmsg.buf[1] = 0x00;
        txmsg.buf[2] = 0x00;
        txmsg.buf[3] = 0x00;

        // Encode Actual_Duty: 39|16@0- (bit 39, 16 bits, little endian, signed) - SET TO 0
        txmsg.buf[4] = 0x00;
        txmsg.buf[5] = 0x00;

        // Encode Actual_InputVoltage: 55|16@0- (bit 55, 16 bits, big-endian, signed)
        // Use received voltage from IVT (convert from mV to V), or default to 0
        uint16_t voltage_to_send = 0;
        if (voltage_received) {
            // Convert from mV to V and ensure it's positive (unsigned)
            int32_t voltage_v = received_voltage_mv / 1000;
            voltage_to_send = (voltage_v < 0) ? 0 : (uint16_t)voltage_v;
        }

        txmsg.buf[6] = (voltage_to_send >> 8) & 0xFF;  // MSB (big-endian)
        txmsg.buf[7] = (voltage_to_send >> 0) & 0xFF;  // LSB (big-endian)

        can1.write(txmsg);

        // Trigger LED on GPIO 36 for sent message (non-blocking)
        digitalWrite(36, HIGH);
        led36_start = millis();
        led36_active = true;

        Serial.print("Sent: ");
        Serial.print(voltage_to_send);
        Serial.println("V");
    }

    // Send BMS charger state command periodically
    if (millis() - last_state_send > STATE_SEND_INTERVAL) {
        last_state_send = millis();
        sendBMSCommand();
    }

    // Check for received CAN messages
    if (can1.read(rxmsg)) {
        // Decode IVT voltage message (ID: 0x522 = 1314 decimal)
        if (rxmsg.id == 0x522) {
            // Decode IVT_Result_U1: 23|32@0- (bit 23, 32 bits, big-endian, signed)
            // This spans bytes 2-5 in the message (big-endian byte order)
            int32_t voltage_raw = 0;
            voltage_raw |= (int32_t)rxmsg.buf[2] << 24;
            voltage_raw |= (int32_t)rxmsg.buf[3] << 16;
            voltage_raw |= (int32_t)rxmsg.buf[4] << 8;
            voltage_raw |= (int32_t)rxmsg.buf[5] << 0;

            // Store the received voltage for delayed transmission (50ms delay)
            pending_voltage_mv = voltage_raw;
            voltage_pending = true;
            voltage_receive_time = millis();  // Record when voltage was received
            last_voltage_time = millis();     // Update timeout timestamp

            // Trigger LED on GPIO 33 for received message (non-blocking)
            digitalWrite(33, HIGH);
            led33_start = millis();
            led33_active = true;

            // Convert from mV to V
            float voltage_volts = voltage_raw / 1000.0;

            Serial.print("Received: ");
            Serial.print(voltage_volts, 3);
            Serial.println("V (will send in 50ms)");
        }
    }
}

// State machine update function
void updateChargerState() {
    ChargerState previous_state = current_state;

    // Priority check: Shutdown condition overrides all others
    if (!shutdown_state) {
        current_state = SHUTDOWN;
    } else {
        // Normal state transitions when shutdown circuit is closed
        switch (current_state) {
            case INITIAL:
                if (shutdown_state) {
                    if (switch_state) {
                        // current_state = CHARGING;  // If switch is already on, go directly to charging
                    } else {
                        current_state = IDLE;  // If switch is off, go to idle
                    }
                }
                break;

            case IDLE:
                if (switch_state) {
                    current_state = CHARGING;
                }
                break;

            case CHARGING:
                if (!switch_state) {
                    current_state = IDLE;
                }
                break;

            case SHUTDOWN:
                // Only exit shutdown when BOTH conditions are met:
                // 1. Shutdown signal goes back to normal (HIGH)
                // 2. Ignition/switch is disengaged (LOW)
                if (shutdown_state && !switch_state) {
                    current_state = IDLE;  // Go to IDLE when shutdown is released AND switch is off
                }
                break;
        }
    }

    // Print state changes
    if (previous_state != current_state) {
        Serial.print("State change: ");
        printState(previous_state);
        Serial.print(" -> ");
        printState(current_state);
        Serial.print(" (Switch: ");
        Serial.print(switch_state);
        Serial.print(", Shutdown: ");
        Serial.print(shutdown_state);
        Serial.print(", Raw Switch: ");
        Serial.print(digitalRead(SWITCH_PIN));
        Serial.print(", Raw Shutdown: ");
        Serial.print(digitalRead(SHUTDOWN_PIN));
        Serial.println(")");
    }
}

// Send BMS command based on current state
void sendBMSCommand() {
    CAN_message_t bms_msg;
    bms_msg.id = 0x83;
    bms_msg.len = 1;

    // Determine command based on state
    switch (current_state) {
        case INITIAL:
        case IDLE:
        case SHUTDOWN:
            bms_msg.buf[0] = 0;  // Open contactors
            break;

        case CHARGING:
            bms_msg.buf[0] = 1;  // Close contactors
            break;
    }

    can1.write(bms_msg);

    // Debug output
    Serial.print("BMS Command: ");
    Serial.print(bms_msg.buf[0]);
    Serial.print(" (State: ");
    printState(current_state);
    Serial.print(", Switch: ");
    Serial.print(switch_state);
    Serial.print(", Shutdown: ");
    Serial.print(shutdown_state);
    Serial.println(")");
}

// Helper function to print state names
void printState(ChargerState state) {
    switch (state) {
        case INITIAL:
            Serial.print("INITIAL");
            break;
        case IDLE:
            Serial.print("IDLE");
            break;
        case CHARGING:
            Serial.print("CHARGING");
            break;
        case SHUTDOWN:
            Serial.print("SHUTDOWN");
            break;
    }
}