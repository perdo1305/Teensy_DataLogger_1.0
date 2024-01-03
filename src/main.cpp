/**
 * @file main.cpp
 * @brief This is the main file for the CAN logger project for L.A.R.T T24e
 * @author Perdu Ferreira
 * @author David Moniz
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

#define ENABLE_INTERRUPT  // uncomment to always log to sd card //fode sd cards
#define OPAMP_PIN 4       // pin for ampop interrupt
#define BUTTON_PIN 5      // pin for button interrupt

#define chipSelect BUILTIN_SDCARD
#define LED1_pin 2
#define LED2_pin 3

#define __LART_DEBUG__  // comentar quando for para o carro

#ifdef __LART_DEBUG__
#define SERIAL_OPEN_TIMEOUT 5000
#else
#define SERIAL_OPEN_TIMEOUT 1200
#endif
//________________________________________________________________________________________________
//__________________________________Function prototypes___________________________________________
void MCU_heartbeat(void);
void CAN_hearbeat(void);

unsigned long processSyncMessage(void);
void RTC_update_by_serial(void);
time_t getTeensy3Time(void);
String milliseconds_calculation(void);

String csvSuffixer(int value, int format);
String csvSuffixer(String value);

void log_to_sdcard(void);
void sendDLstatus(void);
void receiveStart(CAN_message_t msg, uint16_t id_start, uint16_t buf_index);
void receiveMarker(CAN_message_t msg, uint16_t id_start, uint16_t buf_index);
void builDataString(CAN_message_t msg);
//_________________________________________________________________________________________________
//_________________________________________________________________________________________________

// SD card
volatile bool logging_active = false;
String dataString = "";
int file_num_int = 0;  // keep track of the datalog.txt file number
int file_num_plus_one = 0;
File* dataFile_ptr;  // pointer to the datafile to be closed by the interrupt

// Millis variables
unsigned long currentMillis[10] = {};
unsigned long previousMillis[10] = {};

// RTC
unsigned long startMillis = millis();  // start time

// CAN BUS
bool canrx_status = false;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

CAN_message_t rxmsg; // struct to hold received CAN message
CAN_message_t txmsg; // struct to hold sent CAN message

volatile bool DataLoggerActive = false;

void setup() {
    // set the Time library to use Teensy 3.0's RTC to keep time
    setSyncProvider(getTeensy3Time);

    /*############ MCU heartbeat ############*/
    pinMode(LED_BUILTIN, OUTPUT);     // initialize the built-in LED pin as an output
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the built in led
    /*#######################################*/

    unsigned long OV_millis = 0;
    OV_millis = millis();
    Serial.begin(115200);
    while (!Serial) {
        if (millis() - OV_millis > SERIAL_OPEN_TIMEOUT) {
            break;
        }
    }

    // TODO mandar um caracter para contiuar
    //  ############################################# RTC ##############################################
    if (timeStatus() != timeSet) {
        Serial.println("Unable to sync with the RTC");
    } else {
        Serial.println("RTC has set the system time");
    }
    // ############################################# SD card #############################################

    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
            // No SD card, so don't do anything more - stay stuck here
            Serial.println("Card failed, or not present");
            delay(500);
        }
    }
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
        //TODO colocar cabeçalho descritivo,ex started by voltage, rpm, etc
        //TODO nome do ficheiro com descritivo de start

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
}

void loop() {
    RTC_update_by_serial();
    MCU_heartbeat();  // blink the built in led at 3.3Hz
    // CAN_hearbeat();  // blink the can bus rx led at 3.3Hz

    sendDLstatus();  // send the data logger status to the CAN bus

    if (can1.read(rxmsg)) {
        digitalToggle(LED2_pin);

        receiveStart(rxmsg, 0x201, 0);
        receiveMarker(rxmsg, 0x202, 0);
        builDataString(rxmsg);

        canrx_status = true;
        Serial.print(dataString);
    } else {
        canrx_status = false;
        // TODO Log errors too!
    }

    // log the data string to the SD card if logging is active
    if (logging_active && canrx_status) {
        log_to_sdcard();
        digitalToggle(LED1_pin);
    }
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
    if (canrx_status) {
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
    if (msg.id == id_start) {//TODO RPM> tal comecar o logging ou tensao do inversor etc
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
        txmsg.id = 0x200;
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