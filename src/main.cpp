/**
 * @file main.cpp
 * @brief This is the main file for the CAN logger project for L.A.R.T T24e
 * @author Perdu Ferreira
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <U8g2lib.h>
// #include <USBHost_t36.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "CANSART_STR/src/cansart.h"
#include "Watchdog_t4.h"

#define CANSART 0

#if CANSART
frame60 frames60;
frame120 frames120;
frame61 frames61;
frame80 frames80;
HardwareSerial& serialPort = Serial1;
#endif

// #define rx7Pin 28
// #define tx7Pin 29

// SoftwareSerial MySerial7(rx7Pin, tx7Pin);  // RX, TX

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

#define LED_GPIO33 33
#define LED_GPIO34 34
#define LED_GPIO35 35
#define LED_GPIO36 36

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
void Can2_things(void);                                                        // Do the can2 receive things
void Can3_things(void);                                                        // Do the can3 receive things
void StartUpSequence(void);                                                    // LEDs startup sequence

WDT_T4<WDT1> wdt;                                                 // Watchdog timer config
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(SCL, SDA, U8X8_PIN_NONE);  // OLED contructor
//_________________________________________________________________________________________________
//__________________________________Variables______________________________________________________
//_________________________________________________________________________________________________

volatile bool logging_active = false;  // If the data logger is active or not
volatile bool DataLoggerActive = false;
String dataString = "";       // String to be logged to the SD card
int file_num_int = 0;         // Keep track of the datalog.txt file number
int file_num_plus_one = 0;    // Keep track of the datalog.txt file number
File* dataFile_ptr;           // Pointer to the datafile to be closed by the interrupt
bool SD_card_status = false;  // If the SD card is present and initialized

// ________Millis()________
unsigned long currentMillis[10] = {};   // Array to hold the current time
unsigned long previousMillis[10] = {};  // Array to hold the previous time

// ________RTC___________
unsigned long startMillis = millis();  // start time for millis() function in the beginning of the program

// ______CAN BUS_________
bool can1rx_status = false;
bool can2rx_status = false;
bool can3rx_status = false;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

CAN_message_t rxmsg;  // Struct to hold received CAN message
CAN_message_t txmsg;  // Struct to hold sent CAN message

CAN_message_t rxmsg2;  // Struct to hold received CAN message
CAN_message_t txmsg2;  // Struct to hold sent CAN message

CAN_message_t rxmsg3;  // Struct to hold received CAN message
CAN_message_t txmsg3;  // Struct to hold sent CAN message

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
    /*############ MCU heartbeat #######################################################################*/
    pinMode(LED_BUILTIN, OUTPUT);     // initialize the built-in LED pin as an output
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the built in led

    /*##################################################################################################*/

    unsigned long OV_millis = 0;  // Overflow millis
    OV_millis = millis();         // Overflow millis
    // Serial.begin(115200);
    while (!Serial) {
        if (millis() - OV_millis > SERIAL_OPEN_TIMEOUT) {
            break;
        }
    }

    // Hardware
#if CANSART
    setCANSART_Driver(serialPort, (unsigned long)4800);
#endif
    // Serial7.setRX(28);
    // Serial7.begin(9600);  // Telemetria

    // SoftwareSerial
    // pinMode(rx7Pin, INPUT);
    // pinMode(tx7Pin, OUTPUT);
    // MySerial7.begin(9600);

    //  ############################################# RTC ##############################################
    if (timeStatus() != timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // ############################################# SD CARD #############################################

    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(BUILTIN_SDCARD)) {
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
    can2.begin();
    can2.setBaudRate(1000000);
    can3.begin();
    can3.setBaudRate(1000000);
    /*##############################################*/
    DataLoggerActive = true;

    StartUpSequence();

#ifdef __Screen_ON__

    u8x8.setI2CAddress(0x78);
    u8x8.begin();
    displayWelcome();
    delay(2000);
    u8x8.clearDisplay();
#endif
    /*################ WDT #############################################################################*/
    WDT_timings_t config;
    config.trigger = 5;             /* in seconds, 0->128 Warning trigger before timeout */
    config.timeout = 10;            /* in seconds, 0->128 Timeout to reset */
    config.callback = wdtCallback;  // Callback function to be called on timeout
    wdt.begin(config);              // Start the watchdog timer
}

void loop() {
    // Serial1.println("Hello World");
    if (Serial1.available()) {
        Serial.println(Serial1.readString());
    }
#if CANSART
    updateDB(&frames60);
    updateDB(&frames61);
    updateDB(&frames80);
    updateDB(&frames120);
#endif

    wdt.feed();  // Feed the whatchdog timer
    RTC_update_by_serial();
    MCU_heartbeat();  // Blink the built in led at 3.3Hz
    // CAN_hearbeat();  // Blink the can bus rx led at 3.3Hz

    sendDLstatus();  // Send the data logger status to the CAN bus
    Can1_things();   // Do the can1 receive things
<<<<<<< Updated upstream
    // Can2_things();   // Do the can2 receive things

    // delay 2ms
    if (millis() - previousMillis[4] > 2) {
        previousMillis[4] = millis();
        if (logging_active && can1rx_status) {  // Log the data string to the SD card if logging is active and can1rx_status is true
                                                // if (logging_active) {
=======

    if (millis() - previousMillis[4] > 10) {
        previousMillis[4] = millis();
        if (logging_active && can1rx_status) {  // Log the data string to the SD card if logging is active and can1rx_status is true
>>>>>>> Stashed changes
            log_to_sdcard();
            digitalToggle(LED1_pin);
        }
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
struct CarData_MAIN {            // 32 Bytes
    uint16_t RPM = 0;            // 0-65536
    uint8_t VSPD = 0;            // 0-160
    uint8_t APPS1 = 0;           // 0-100
    uint8_t BRAKE = 0;           // 0-1
    uint8_t DBWPOS = 0;          // 0-100
    uint8_t LAMBDA = 0;          // 0-2
    uint8_t OILT = 0;            // 0-160
    uint8_t OILP = 0;            // 0-12
    uint16_t ENGT1 = 0;          // 0-1100
    uint16_t ENGT2 = 0;          // 0-1100
    uint8_t BATV = 0;            // 0-20
    uint8_t IAT = 0;             // 0-167 subtrair 40 no labView
    uint8_t MAP = 0;             // 0-4
    uint16_t CLT = 0;            // 0-290 subtrair 40 no labView
    uint8_t FUELP = 0;           // 0-1
    uint8_t IGNANG = 0;          // 0-20
    uint8_t CBUSLD = 0;          // 0-100
    uint8_t LAMCORR = 0;         // 75-125
    uint8_t ECUT = 0;            // 0-4
    uint8_t DBWTRGT = 0;         // 0-100
    uint8_t ACCX = 0;            // 0-20
    uint8_t DataLoggerSTAT = 0;  // 0-100
    uint8_t GearValue = 0;       // 0-1
    uint8_t ROLL = 0;            // 75-125
    uint8_t PITCH = 0;           // 0-4
    uint8_t YAW = 0;             // 0-100
    uint8_t LOGSTAT = 0;         // 0-20
    char inicio = 10;            // END
};
struct CarData_MAIN carDataMain;

void sendTelemetry() {
}

#endif

// TODO Stearing angle
void Can1_things() {
    CAN_error_t error;
    if (can1.error(error, 0)) {
        Serial.println("CAN1 ERROR");
        can1rx_status = false;
        digitalWrite(LED_GPIO34, LOW);  // turn off the can bus rx led
    } else {
        if (can1.read(rxmsg)) {
            switch (rxmsg.id) {
                case 0x60:
#if CANSART
                    frames60.DATA1 = rxmsg.buf[0];
                    frames60.DATA2 = rxmsg.buf[1];
                    frames60.DATA3 = rxmsg.buf[2];
                    frames60.DATA4 = rxmsg.buf[3];
                    frames60.DATA5 = rxmsg.buf[4];
                    frames60.DATA6 = rxmsg.buf[5];
                    frames60.DATA7 = rxmsg.buf[6];
                    frames60.DATA8 = rxmsg.buf[7];
#endif
                    /*
                        carDataMain.RPM = rxmsg.buf[0] << 8 | rxmsg.buf[1];
                        carDataMain.OILP = rxmsg.buf[2] << 8 | rxmsg.buf[3];
                        carDataMain.OILT = rxmsg.buf[4] << 8 | rxmsg.buf[5];
                        carDataMain.BATV = rxmsg.buf[6] << 8 | rxmsg.buf[7];
                        */
                    break;
                case 0x61:
                    // carDataMain.VSPD = rxmsg.buf[0] << 8 | rxmsg.buf[1];
#if CANSART
                    frames61.DATA1 = rxmsg.buf[0];
                    frames61.DATA2 = rxmsg.buf[1];
                    frames61.DATA3 = rxmsg.buf[2];
                    frames61.DATA4 = rxmsg.buf[3];
                    frames61.DATA5 = rxmsg.buf[4];
                    frames61.DATA6 = rxmsg.buf[5];
                    frames61.DATA7 = rxmsg.buf[6];
                    frames61.DATA8 = rxmsg.buf[7];
#endif
                    break;
                case 0x62:
                    break;
                case 0x80:
#if CANSART
                    frames80.DATA1 = rxmsg.buf[0];
                    frames80.DATA2 = rxmsg.buf[1];
                    frames80.DATA3 = rxmsg.buf[2];
                    frames80.DATA4 = rxmsg.buf[3];
                    frames80.DATA5 = rxmsg.buf[4];
                    frames80.DATA6 = rxmsg.buf[5];
                    frames80.DATA7 = rxmsg.buf[6];
                    frames80.DATA8 = rxmsg.buf[7];
#endif
                    break;
                case 0x200:
#if CANSART
                    frames120.DATA1 = rxmsg.buf[0];
                    frames120.DATA2 = rxmsg.buf[1];
                    frames120.DATA3 = rxmsg.buf[2];
                    frames120.DATA4 = rxmsg.buf[3];
                    frames120.DATA5 = rxmsg.buf[4];
                    frames120.DATA6 = rxmsg.buf[5];
                    frames120.DATA7 = rxmsg.buf[6];
                    frames120.DATA8 = rxmsg.buf[7];
#endif
                    break;
                case 0x202:
                    // receiveMarker(rxmsg, 0x202, 0);
                default:
                    // add ids to filter
                    break;
            }

            digitalToggle(LED_GPIO34);  // toggle the can bus rx led
            builDataString(rxmsg);      // build the data string to be logged to the SD card
            can1rx_status = true;       // set the can1rx_status to true
            Serial.print(rxmsg.id);     // print the data string to the serial port
        }
    }
}

void Can2_things() {
    CAN_error_t error;
    if (can2.error(error, 0)) {
        can2rx_status = false;
        digitalWrite(LED_GPIO35, LOW);  // turn off the can bus rx led
    } else {
        if (can2.read(rxmsg2)) {
            digitalToggle(LED_GPIO35);  // toggle the can bus rx led
            // builDataString(rxmsg2);    // build the data string to be logged to the SD card
            can2rx_status = true;     // set the can1rx_status to true
            Serial.print(rxmsg2.id);  // print the data string to the serial port
        }
    }
}

void Can3_things() {
    CAN_error_t error;
    if (can3.error(error, 0)) {
        can3rx_status = false;
        digitalWrite(LED_GPIO36, LOW);  // turn off the can bus rx led
    } else {
        if (can3.read(rxmsg3)) {
            digitalToggle(LED_GPIO36);  // toggle the can bus rx led
            // builDataString(rxmsg3);    // build the data string to be logged to the SD card
            can3rx_status = true;     // set the can1rx_status to true
            Serial.print(rxmsg3.id);  // print the data string to the serial port
        }
    }
}

void StartUpSequence() {
    pinMode(LED_GPIO33, OUTPUT);
    pinMode(LED_GPIO34, OUTPUT);
    pinMode(LED_GPIO35, OUTPUT);
    pinMode(LED_GPIO36, OUTPUT);

    digitalWrite(LED_GPIO33, HIGH);
    delay(80);
    digitalWrite(LED_GPIO34, HIGH);
    delay(80);
    digitalWrite(LED_GPIO35, HIGH);
    delay(80);
    digitalWrite(LED_GPIO36, HIGH);
    delay(80);

    digitalWrite(LED_GPIO33, LOW);
    delay(80);
    digitalWrite(LED_GPIO34, LOW);
    delay(80);
    digitalWrite(LED_GPIO35, LOW);
    delay(80);
    digitalWrite(LED_GPIO36, LOW);
    delay(80);

    digitalWrite(LED_GPIO33, HIGH);
    delay(80);
    digitalWrite(LED_GPIO34, HIGH);
    delay(80);
    digitalWrite(LED_GPIO35, HIGH);
    delay(80);
    digitalWrite(LED_GPIO36, HIGH);
    delay(80);

    digitalWrite(LED_GPIO33, LOW);
    delay(80);
    digitalWrite(LED_GPIO34, LOW);
    delay(80);
    digitalWrite(LED_GPIO35, LOW);
    delay(80);
    digitalWrite(LED_GPIO36, LOW);
}