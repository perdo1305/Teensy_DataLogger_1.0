/**
 * @file main.cpp
 * @brief This is the main file for the CAN logger project for L.A.R.T T24e
 * @author Perdu Ferreira
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <U8g2lib.h>
// #include <USBHost_t36.h>
#include <Wire.h>

#include "Watchdog_t4.h"

//________________________________________________________________________________________________
//__________________________________Defines_________________________________________________________
//________________________________________________________________________________________________
#define DataLogger_CanId 0x200

#define ENABLE_INTERRUPT  // uncomment to always log to sd card //fode sd cards
#define OPAMP_PIN 4       // pin for ampop interrupt
#define BUTTON_PIN 5      // pin for button interrupt

#define chipSelect BUILTIN_SDCARD
#define LED1_pin 2
#define LED2_pin 3

#define __LART_DEBUG__  // comentar quando for para o carro

#ifdef __LART_DEBUG__
#define SERIAL_OPEN_TIMEOUT 5000
#define __Screen_ON__
#define debug_begin(x) Serial.begin(x)
#define debug_println(x) Serial.println(x)
#define debug_print(x) Serial.print(x)
#else
#define SERIAL_OPEN_TIMEOUT 1200
#endif

// #define __Telemetria_ON__  // descomentar quando for para o carro

//________________________________________________________________________________________________
//__________________________________Function prototypes___________________________________________
//________________________________________________________________________________________________
void MCU_heartbeat(void);  // Blink the built in led at 3.3Hz
void CAN_hearbeat(void);   // Blink the can bus rx led at 3.3Hz

unsigned long processSyncMessage(void);
void RTC_update_by_serial(void);        // Update the RTC by serial port if available
time_t getTeensy3Time(void);            // Function prototype for getTeensy3Time()
String milliseconds_calculation(void);  // Get string in the format of "HH:MM:SS:ms"

String csvSuffixer(int value, int format);  // Formats the string to be logged to the SD card
String csvSuffixer(String value);           // Formats the string to be logged to the SD card

void log_to_sdcard(void);                                                      // Log the data string to the SD card
void sendDLstatus(void);                                                       // Send the data logger status to the CAN bus
void receiveStart(CAN_message_t msg, uint16_t id_start, uint16_t buf_index);   // Start/Stop the data logger by receiving a CAN frame
void receiveMarker(CAN_message_t msg, uint16_t id_start, uint16_t buf_index);  // Receive a marker from the CAN bus
void builDataString(CAN_message_t msg);                                        // Build the data string to be logged to the SD card
void displayWelcome(void);                                                     // Display the welcome message on the OLED
void displayDataLoggerStatus(void);                                            // Display the data logger status on the OLED
void Can1_things(void);                                                        // Do the can1 receive things

WDT_T4<WDT1> wdt;                                                 // Watchdog timer config
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(SCL, SDA, U8X8_PIN_NONE);  // OLED contructor
//_________________________________________________________________________________________________
//__________________________________Variables_______________________________________________________
//_________________________________________________________________________________________________

volatile bool logging_active = false;  // If the data logger is active or not
String dataString = "";                // String to be logged to the SD card
int file_num_int = 0;                  // Keep track of the datalog.txt file number
int file_num_plus_one = 0;             // Keep track of the datalog.txt file number
File* dataFile_ptr;                    // Pointer to the datafile to be closed by the interrupt
bool SD_card_status = false;           // If the SD card is present and initialized

// millis() variables
unsigned long currentMillis[10] = {};   // Array to hold the current time
unsigned long previousMillis[10] = {};  // Array to hold the previous time

// ________RTC___________
unsigned long startMillis = millis();  // start time for millis() function in the beginning of the program

// ______CAN BUS_________
bool can1rx_status = false;
bool can2rx_status = false;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; //Uncomment if using can2

CAN_message_t rxmsg;  // Struct to hold received CAN message
CAN_message_t txmsg;  // Struct to hold sent CAN message

volatile bool DataLoggerActive = false;

//______Telemetria_______
float CAN_Bus_Data[50];
IntervalTimer TelemetryTimer;
void sendTelemetry(void);

void wdtCallback() {
    Serial.println("FEED THE DOG SOON, OR RESET!");
}

void setup() {
    setSyncProvider(getTeensy3Time);  // Set the Time library to use Teensy 3.0's RTC to keep time
#ifdef __Telemetria_ON__
    TelemetryTimer.begin(sendTelemetry, 20000);  // interval timer for telemetry
#endif
    /*############ MCU heartbeat ############*/
    pinMode(LED_BUILTIN, OUTPUT);     // initialize the built-in LED pin as an output
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the built in led
    /*################ WDT ##################*/
    WDT_timings_t config;
    config.trigger = 5;             /* in seconds, 0->128 Warning trigger before timeout */
    config.timeout = 10;            /* in seconds, 0->128 Timeout to reset */
    config.callback = wdtCallback;  // Callback function to be called on timeout
    wdt.begin(config);              // Start the watchdog timer
    /*#########################################*/
    unsigned long OV_millis = 0;  // Overflow millis
    OV_millis = millis();         // Overflow millis
    Serial.begin(115200);
    while (!Serial) {
        if (millis() - OV_millis > SERIAL_OPEN_TIMEOUT) {
            break;
        }
    }
    //  ############################################# RTC ##############################################
    if (timeStatus() != timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // ############################################# SD CARD #############################################

    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
            // No SD card, so don't do anything more - stay stuck here
            SD_card_status = false;
            u8x8.setI2CAddress(0x78);
            u8x8.begin();
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.drawString(4, 2, "SD card");
            u8x8.drawString(4, 3, "failed!");
            Serial.println("Card failed, or not present");
            delay(500);
        }
    }
    SD_card_status = true;
    Serial.println("card initialized.");
    // read a number from a file and increment it
    File dataFile = SD.open("config.txt", FILE_READ);
    if (dataFile) {
        char str[10] = {};
        int i = 0;
        while (dataFile.available()) {
            // Serial.write(file.read());
            // NUM_file=file.read();
            str[i] = dataFile.read();
            i++;
        }
        file_num_int = 0;
        file_num_int = atoi(str);
        file_num_plus_one = (int)file_num_int + 1;
        // TODO colocar cabeçalho descritivo,ex started by voltage, rpm, etc
        // TODO nome do ficheiro com descritivo de start

        dataFile.close();

    } else {
        // if the file isn't open, pop up an error:
        Serial.println("error opening config.txt");
    }
    // write the incremented number to the file
    dataFile = SD.open("config.txt", FILE_WRITE);

    if (dataFile) {
        char buffer[20] = {};
        sprintf(buffer, "%d", file_num_plus_one);
        // delete the old content
        dataFile.truncate(0);
        dataFile.print(buffer);
        dataFile.close();
    } else {
        // if the file isn't open, pop up an error:
        Serial.println("error opening config.txt");
    }

    // ###################################################################################################

    Serial.println("█▀▀ ▄▀█ █▄░█   █▄▄ █░█ █▀   █▀▄ ▄▀█ ▀█▀ ▄▀█   █░░ █▀█ █▀▀ █▀▀ █▀▀ █▀█");
    Serial.println("█▄▄ █▀█ █░▀█   █▄█ █▄█ ▄█   █▄▀ █▀█ ░█░ █▀█   █▄▄ █▄█ █▄█ █▄█ ██▄ █▀▄");

    /*############### Record Button ################*/
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // set the button pin as input with pullup resistor

#ifdef ENABLE_INTERRUPT
    attachInterrupt(  // attach interrupt to the button pin
        digitalPinToInterrupt(BUTTON_PIN), []() {
            // debounce
            if (millis() - previousMillis[2] < 200) {
                return;
            }
            previousMillis[2] = millis();

            logging_active = !logging_active;
            Serial.println(logging_active ? "Logging started" : "Logging stopped");
        },
        FALLING);

#else
    logging_active = true;
#endif

    attachInterrupt(  // attach interrupt to the button pin
        digitalPinToInterrupt(OPAMP_PIN), []() {
            if (dataFile_ptr != nullptr)
                dataFile_ptr->close();
        },
        RISING);

    /*##############################################*/

    /*#################### CAN #####################*/
    can1.begin();
    can1.setBaudRate(1000000);
    /*##############################################*/

    /*#################### Heartbeat LEDS #################*/
    pinMode(LED1_pin, OUTPUT);  // logging active led
    pinMode(LED2_pin, OUTPUT);  // can bus rx led
    //-----------------------------------------------------
    digitalWrite(LED1_pin, HIGH);  // turn on the logging active led
    digitalWrite(LED2_pin, HIGH);  // turn on the can bus rx led
    /*#####################################################*/
    DataLoggerActive = true;

#ifdef __Screen_ON__

    u8x8.setI2CAddress(0x78);
    u8x8.begin();
    displayWelcome();
    delay(2000);
    u8x8.clearDisplay();
#endif
}

void loop() {
    wdt.feed();  // Feed the whatchdog timer
    RTC_update_by_serial();
    MCU_heartbeat();  // Blink the built in led at 3.3Hz
    // CAN_hearbeat();  // Blink the can bus rx led at 3.3Hz

    sendDLstatus();  // Send the data logger status to the CAN bus
    Can1_things();   // Do the can1 receive things

    if (logging_active && can1rx_status) {  // Log the data string to the SD card if logging is active and can1rx_status is true
        log_to_sdcard();
        digitalToggle(LED1_pin);
    }

#ifdef __Screen_ON__
    if (millis() - previousMillis[3] > 500) {
        previousMillis[3] = millis();
        displayDataLoggerStatus();  // Display the data logger status on the OLED
    }
#endif
}

/**
 * @brief Blink the built in led at 3.3Hz
 */
void MCU_heartbeat() {
    currentMillis[1] = millis();
    if (currentMillis[1] - previousMillis[1] > 300) {
        previousMillis[1] = currentMillis[1];
        digitalToggle(LED_BUILTIN);
    }
}

/**
 * @brief Blink the can bus rx led at 3.3Hz
 */
void CAN_hearbeat() {
    if (can1rx_status) {
        currentMillis[2] = millis();
        if (currentMillis[2] - previousMillis[2] > 300) {
            previousMillis[2] = currentMillis[2];
            digitalToggle(LED2_pin);
        }
    } else {
        digitalWrite(LED2_pin, LOW);
    }
}

//_________________________________________________________________________________________________
// ############################################## RTC #############################################
//_________________________________________________________________________________________________

String milliseconds_calculation() {
    // get string in the format of "HH:MM:SS:ms"
    String time_str = "";
    // milliseconds using millis() function and seconds using RTC
    unsigned long currentMillis = millis();
    unsigned long elapsedMillis = currentMillis - startMillis;
    time_str += String(elapsedMillis % 1000);
    return time_str;
}

void RTC_update_by_serial() {
    if (Serial.available()) {
        time_t t = processSyncMessage();
        if (t != 0) {
            Teensy3Clock.set(t);  // set the RTC
            setTime(t);
        }
    }
}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER "T"  // Header tag for serial time sync message

unsigned long processSyncMessage() {
    unsigned long pctime = 0L;
    const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013

    if (Serial.find(TIME_HEADER)) {
        pctime = Serial.parseInt();
        return pctime;
        if (pctime < DEFAULT_TIME) {  // check the value is a valid time (greater than Jan 1 2013)
            pctime = 0L;              // return 0 to indicate that the time is not valid
        }
    }
    return pctime;
}

//_________________________________________________________________________________________________
// ################################################################################################
//_________________________________________________________________________________________________

/**
 * @brief log the data string to the SD card
 */
void log_to_sdcard() {
    char file_name[20] = {};
    sprintf(file_name, "datalog_%d.csv", file_num_int);

    File dataFile = SD.open(file_name, FILE_WRITE);
    dataFile_ptr = &dataFile;
    if (dataFile) {
        dataFile.print(dataString);
        dataFile.close();
        dataFile_ptr = nullptr;
        // print to the serial port too:
        // Serial.println(dataString);
    } else {
        // if the file isn't open, pop up an error:
        Serial.println("error opening datalog.txt");
    }

    dataString = "";
}

/**
 * @brief Build the data string to be logged to the SD card
 */
void builDataString(CAN_message_t msg) {
    dataString += String(hour());
    dataString += csvSuffixer(minute());
    dataString += csvSuffixer(second());
    dataString += csvSuffixer(milliseconds_calculation());
    dataString += csvSuffixer(rxmsg.id, HEX);
    dataString += csvSuffixer(rxmsg.len, HEX);
    for (int i = 0; i < rxmsg.len; i++) {
        dataString += csvSuffixer(rxmsg.buf[i], HEX);
    }
    dataString += "\n";
}

/**
 * @brief convert input to a string and prepend a ";" to it
 * @param value value to be converted to string
 * @param format format of the string in decimal format
 * @return String
 * @author David Moniz
 */
String csvSuffixer(int value, int format = DEC) {
    auto ret = ";" + String(value, format);
    return ret;
}

/**
 * @brief convert input to a string and prepend a ";" to it
 * @param value value to be converted to string
 * @return String
 * @author David Moniz
 */
String csvSuffixer(String value) {
    auto ret = ";" + value;
    return ret;
}

/**
 * @brief Start/Stop the data logger by receiving a CAN frame
 * @param msg CAN struct
 * @param id_start CAN id to start the data logger
 * @param buf_index index of the buffer to check if the data logger is active
 * @return void
 */
void receiveStart(CAN_message_t msg, uint16_t id_start, uint16_t buf_index) {
    if (msg.id == id_start) {  // TODO RPM> tal comecar o logging ou tensao do inversor etc
        msg.buf[buf_index] >= 1 ? logging_active = true : logging_active = false;
    }
}

/**
 * @brief Receive a marker from the CAN bus
 * @param msg CAN struct
 * @return void
 */
void receiveMarker(CAN_message_t msg, uint16_t id_start, uint16_t buf_index) {
    if (msg.id == id_start) {
        // TODO marker
    }
}

/**
 * @brief Send the data logger status to the CAN bus
 */
void sendDLstatus() {
    currentMillis[0] = millis();
    if (currentMillis[0] - previousMillis[0] > 5) {
        previousMillis[0] = currentMillis[0];
        // DATA LOGGER STATUS
        txmsg.id = DataLogger_CanId;
        txmsg.len = 8;
        txmsg.buf[0] = DataLoggerActive;  // 1 = data logger active, 0 = data logger inactive
        txmsg.buf[1] = logging_active;    // 1 = logging active, 0 = logging inactive
        txmsg.buf[2] = 1;
        txmsg.buf[3] = 2;
        txmsg.buf[4] = 3;
        txmsg.buf[5] = 4;
        txmsg.buf[6] = 5;
        txmsg.buf[7] = 6;

        can1.write(txmsg);
    }
}

void displayWelcome() {
    // u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setInverseFont(1);
    u8x8.drawString(5, 0, "L.A.R.T");
    u8x8.setInverseFont(0);
    u8x8.drawString(2, 2, "Data Logger");
    u8x8.drawString(6, 3, "v1.1");
    u8x8.drawString(3, 5, "By : Perdu");
}

void displayDataLoggerStatus() {
    // u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setInverseFont(1);
    u8x8.drawString(0, 0, "## DL Status ##");
    u8x8.setInverseFont(0);
    static int previous_can1rx_status = -1;
    if (previous_can1rx_status != can1rx_status) {
        u8x8.clearLine(1);
        if (can1rx_status) {
            u8x8.drawString(0, 1, "CAN1: OK");
        } else {
            u8x8.drawString(0, 1, "CAN1: ERROR");
        }
        previous_can1rx_status = can1rx_status;
    }

    static int previous_can2rx_status = -1;
    if (previous_can2rx_status != can2rx_status) {
        u8x8.clearLine(2);
        if (can2rx_status)
            u8x8.drawString(0, 2, "CAN2: OK");
        else
            u8x8.drawString(0, 2, "CAN2: ---");
        previous_can2rx_status = can2rx_status;
    }
    static int previous_logging_active = -1;
    if (previous_logging_active != logging_active) {
        u8x8.clearLine(3);
        if (logging_active) {
            u8x8.drawString(0, 3, "Logging: ON");
        } else {
            u8x8.drawString(0, 3, "Logging: OFF");
        }
        previous_logging_active = logging_active;
    }
    static int previous_SD_card_status = -1;
    if (previous_SD_card_status != SD_card_status) {
        u8x8.clearLine(4);
        if (SD_card_status) {
            u8x8.drawString(0, 4, "SD card: OK");
        } else {
            u8x8.drawString(0, 4, "SD card: ERROR");
        }
        previous_SD_card_status = SD_card_status;
    }
    char time_str[15];
    sprintf(time_str, "Time: %02d:%02d:%02d", hour(), minute(), second());
    u8x8.drawString(0, 5, time_str);
    // file name
    static int previous_file_num_int = -1;
    if (previous_file_num_int != file_num_int) {
        u8x8.clearLine(6);
        char file_name[20] = {};
        sprintf(file_name, "File num: %d", file_num_int);
        u8x8.drawString(0, 6, file_name);
        previous_file_num_int = file_num_int;
    }
}
#ifdef __Telemetria_ON__
struct CarData_MAIN {
    bool DataLogger_STAT = 0;      // 0-1
    bool VCU_STAT = 0;             // 0-1
    bool TCU_STAT = 0;             // 0-1
    bool BMS_STAT = 0;             // 0-1
    bool INVERTER_STAT = 0;        // 0-1
    bool PDM_STAT = 0;             // 0-1
    bool STEERING_WHEEL_STAT = 0;  // 0-1
    bool IMU_STAT = 0;             // 0-1
    bool DYNAMICS_FRONT_STAT = 0;  // 0-1
    bool DYNAMICS_BACK_STAT = 0;   // 0-1
    bool ACU_STAT = 0;             // 0-1
    bool ALC_STAT = 0;             // 0-1
    bool AMS_STAT = 0;             // 0-1
    bool STEERING_ANGLE_STAT = 0;  // 0-1

    
};
struct CarData_MAIN T24_E;

void sendTelemetry() {
    T24_E.RPM = CAN_Bus_Data[1];
    T24_E.APPS1 = CAN_Bus_Data[2];
    T24_E.IAT = CAN_Bus_Data[3];
    T24_E.MAP = CAN_Bus_Data[4];
    T24_E.CLT = CAN_Bus_Data[5];
    T24_E.VSPD = CAN_Bus_Data[6];
    T24_E.OILT = CAN_Bus_Data[7];
    T24_E.OILP = CAN_Bus_Data[8] * 10;
    T24_E.FUELP = CAN_Bus_Data[9];
    T24_E.BATV = CAN_Bus_Data[10] * 10;
    T24_E.IGNANG = CAN_Bus_Data[11];
    T24_E.LAMBDA = CAN_Bus_Data[13] * 10;
    T24_E.ENGT1 = CAN_Bus_Data[15];
    T24_E.ENGT2 = CAN_Bus_Data[16];
    T24_E.CBUSLD = CAN_Bus_Data[17];
    T24_E.ECUT = CAN_Bus_Data[18];
    T24_E.DBWPOS = CAN_Bus_Data[19];
    T24_E.DBWTRGT = CAN_Bus_Data[20];

    // carDataMain.DataLoggerSTAT = Logger_Status;
    carDataMain.GearValue = CAN_Bus_Data[44];

    carDataMain.PITCH = CAN_Bus_Data[40];
    carDataMain.ROLL = CAN_Bus_Data[41];
    carDataMain.YAW = CAN_Bus_Data[42];
    /*
    if (brakeSensor < 186) {
        carDataMain.BRAKE = 1;
    } else {
        carDataMain.BRAKE = 0;
    }
    carDataMain.LOGSTAT = Logger_Status;
    if (Serial1.available() > 0) {
        if (Serial1.readString().toInt() == 30) {
            if (D_Logger == 0) {
                D_Logger = 255;
            } else {
                D_Logger = 0;
            }
        }
    }
    */
    Serial7.write((uint8_t*)&carDataMain, sizeof carDataMain);
}
#endif

void Can1_things() {
    if (can1.read(rxmsg)) {
        /*
        switch (rxmsg.id) {
            case 0x201:
                receiveStart(rxmsg, 0x201, 0);
                break;
            case 0x202:
                receiveMarker(rxmsg, 0x202, 0);
            default:
                // add ids to filter
                break;
        }
        */
        digitalToggle(LED2_pin);   // toggle the can bus rx led
        builDataString(rxmsg);     // build the data string to be logged to the SD card
        can1rx_status = true;      // set the can1rx_status to true
        Serial.print(dataString);  // print the data string to the serial port
    } else {
        can1rx_status = false;
        digitalWrite(LED2_pin, LOW);  // turn off the can bus rx led
    }
}

// write all the data in the sd card to the usb stick
/*
USBHost myusb;
USBDrive myDrive(myusb);

USBFilesystem myFiles(myusb);
*/
void write_to_usb() {
    // TODO
}