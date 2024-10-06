/**
 * @file main.cpp
 * @brief This is the main file for the CAN logger project for L.A.R.T T24e
 * @author Pedro Ferreira
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <U8g2lib.h>

#include <array>
#include <string>
// #include <USBHost_t36.h>

#include <Wire.h>

#include "variables.h"

// #include "cansart.h"
#include "../Can-Header-Map/CAN_pwtdb.h"
#include "RF24.h"  // for nRF24L01
#include "SerialTransfer.h"
#include "Watchdog_t4.h"

#define CE_PIN 9
#define CSN_PIN 10

// RF24 radio(CE_PIN, CSN_PIN);

HV500_t hv500;

//________________________________________________________________________________________________
//__________________________________Defines_________________________________________________________
//________________________________________________________________________________________________
#define DataLogger_CanId 0x200

#define ENABLE_INTERRUPT  // uncomment to always log to sd card //fode sd cards
#define ENABLE_EXTERNAL_BUTTON 0

#define OPAMP_PIN 4   // pin for ampop interrupt
#define BUTTON_PIN 5  // pin for button interrupt

#define __LART_DEBUG__  // comentar quando for para o carro

#ifdef __LART_DEBUG__
#define SERIAL_OPEN_TIMEOUT 3000
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

#define COCKPIT_BUTTON 37
#define Button_Cockpit 28

// #define __Telemetria_ON__  // descomentar quando for para o carro

//________________________________________________________________________________________________
//__________________________________Function prototypes___________________________________________
//________________________________________________________________________________________________
void MCU_heartbeat(void);  // Blink the built in led at 3.3Hz

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
void builDataString(CAN_message_t msg, uint8_t can_identifier);                // Build the data string to be logged to the SD card
void displayWelcome(void);                                                     // Display the welcome message on the OLED
void displayDataLoggerStatus(void);                                            // Display the data logger status on the OLED

void Can1_things(void);              // Do the can1 receive things
void Can2_things(void);              // Do the can2 receive things
void Can3_things(void);              // Do the can3 receive things
void decodeDataCan1(CAN_message_t);  // Decode the data from the can1 bus
void decodeDataCan2(CAN_message_t);  // Decode the data from the can2 bus
void decodeDataCan3(CAN_message_t);  // Decode the data from the can3 bus

void StartUpSequence(void);  // LEDs startup sequence
void BUTTON_LED_TASK(void);  // Blink the button led
void createNewFile(void);    // Create a new file to log to the SD card
void FadeLed(void);          // Fade the led

WDT_T4<WDT1> wdt;                                                 // Watchdog timer config
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(SCL, SDA, U8X8_PIN_NONE);  // OLED contructor
//_________________________________________________________________________________________________
//__________________________________Variables______________________________________________________
//_________________________________________________________________________________________________

volatile bool logging_active = false;  // If the logging is active or not
unsigned long loggingStartTime = 0;
volatile bool DataLoggerActive = false;  // If the data logger is active or not
std::string FILE_NAME;                   // Name of the file to be logged to the SD card
constexpr char folderName[] = "logs";    // Folder name to store the logs
String dataString = "";                  // String to be logged to the SD card

int file_count = 0;            // Count how many files are in the SD card
int file_num_int = 0;          // Keep track of the datalog.txt file number
int file_num_plus_one = 0;     // Keep track of the datalog.txt file number
File* dataFile_ptr = nullptr;  // Pointer to the datafile to be closed by the interrupt
bool SD_card_status = false;   // If the SD card is present and initialized

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

void CreateHeader(void);  // Create the header of the file to be logged to the SD card
void Createfooter(void);  // Create the footer of the file to be logged to the SD card

void wdtCallback() {
    Serial.println("FEED THE DOG SOON, OR RESET!");
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
    setSyncProvider(getTeensy3Time);  // Set the Time library to use Teensy 3.0's RTC to keep time

    pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin as an output

    digitalWrite(LED_BUILTIN, HIGH);  // turn on the built in led

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
            delay(2000);
        }
    }

    SD_card_status = true;
    Serial.println("card initialized.");

    /*
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
    }*/

    // ###################################################################################################

    Serial.println("█▀▀ ▄▀█ █▄░█   █▄▄ █░█ █▀   █▀▄ ▄▀█ ▀█▀ ▄▀█   █░░ █▀█ █▀▀ █▀▀ █▀▀ █▀█");
    Serial.println("█▄▄ █▀█ █░▀█   █▄█ █▄█ ▄█   █▄▀ █▀█ ░█░ █▀█   █▄▄ █▄█ █▄█ █▄█ ██▄ █▀▄");

    /*############### Record Button ################*/
    pinMode(BUTTON_PIN, INPUT);  //
    pinMode(COCKPIT_BUTTON, OUTPUT);
    digitalWrite(COCKPIT_BUTTON, LOW);

    pinMode(OPAMP_PIN, INPUT_PULLDOWN);
    pinMode(Button_Cockpit, INPUT_PULLUP);

    /*############### Update RTC by serial port ################*/
    RTC_update_by_serial();

    attachInterrupt(  // attach interrupt to opamp protection pin
        digitalPinToInterrupt(OPAMP_PIN), []() {
            if (dataFile_ptr != nullptr) {
                Createfooter();
                dataFile_ptr->close();
            }
        },
        RISING);

    attachInterrupt(  // attach interrupt to button pin on the pcb
        digitalPinToInterrupt(BUTTON_PIN), []() {
            if (!logging_active) {  // not logging -> start logging
                createNewFile();
                CreateHeader();
            } else {  // logging -> stop logging
                Createfooter();
            }
            logging_active = !logging_active;
        },
        FALLING);

    /*##############################################*/

    /*#################### CAN INIT ################*/
    can1.begin();
    can1.setBaudRate(1000000);
    can2.begin();
    can2.setBaudRate(1000000);
    can3.begin();
    can3.setBaudRate(250000);
    /*##############################################*/

#ifdef __Screen_ON__
    u8x8.setI2CAddress(0x78);
    u8x8.begin();
    displayWelcome();
    delay(300);
    u8x8.clearDisplay();
#endif
    /*################ WDT #############################################################################*/
    WDT_timings_t config;
    config.trigger = 5;             /* in seconds, 0->128 Warning trigger before timeout */
    config.timeout = 10;            /* in seconds, 0->128 Timeout to reset */
    config.callback = wdtCallback;  // Callback function to be called on timeout
    wdt.begin(config);              // Start the watchdog timer
    /*###################################################################################################*/

    StartUpSequence();
    DataLoggerActive = true;
    logging_active = true;
    loggingStartTime = millis();
    createNewFile();
    CreateHeader();
}

void loop() {
    wdt.feed();  // Feed the whatchdog timer

    MCU_heartbeat();  // Blink the built in led at 3.3Hz

    Can1_things();  // Do the can1 receive things
    // Can2_things();  // Do the can2 receive things
    // Can3_things();  // Do the can3 receive things

    BUTTON_LED_TASK();  // Blink the button led on the cockpit

    sendDLstatus();  // Send the data logger status to the CAN bus

    /*
    if ((hv500.InputVoltage > 60) && !logging_active) {
        createNewFile();
        logging_active = true;  // start logging
    }
    */

    if (millis() - previousMillis[4] > 15) {  // log to sd card every 15ms
        previousMillis[4] = millis();
        if (logging_active) {
            log_to_sdcard();
            digitalToggle(LED_GPIO33);
        } else {
            digitalWrite(LED_GPIO33, LOW);
        }
    }

#ifdef __Screen_ON__
    if (millis() - previousMillis[3] > 333) {
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
    // fade the led
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
    if (dataString == "" || !logging_active) {
        return;
    }

    File dataFile = SD.open(FILE_NAME.c_str(), FILE_WRITE);

    if (dataFile) {
        dataFile.print(dataString.c_str());
        dataFile.close();
    } else {
        // Serial.println("Error opening LOG file");
    }

    dataString = "";
}
void builDataString(CAN_message_t msg, uint8_t can_identifier) {
    // dataString += String(hour());
    // dataString += ":";
    // dataString += String(minute());
    // dataString += ":";
    // dataString += String(second());
    // dataString += ".";
    // dataString += String(milliseconds_calculation());

    // Calculate elapsed time since logging started
    unsigned long elapsedTime = millis() - loggingStartTime;
    unsigned long seconds = elapsedTime / 1000;
    unsigned long milliseconds = elapsedTime % 1000;

    // Format elapsed time as SS.sss
    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "  %02lu.%03lu", seconds, milliseconds);

    dataString += timeStr;
    // if the can id is extended the id needs to have a x in the end of it
    if (msg.flags.extended) {
        dataString += csvSuffixer(can_identifier, HEX) + "x";
    } else {
        dataString += csvSuffixer(can_identifier, HEX);
    }

    dataString += csvSuffixer(msg.id, HEX);
    dataString += csvSuffixer(msg.len, HEX);
    for (int i = 0; i < msg.len; i++) {
        dataString += csvSuffixer(msg.buf[i], HEX);
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
    auto ret = " " + String(value, format);
    return ret;
}

/**
 * @brief convert input to a string and prepend a ";" to it
 * @param value value to be converted to string
 * @return String
 * @author David Moniz
 */
String csvSuffixer(String value) {
    auto ret = " " + value;
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
    if (msg.id == id_start) {  // TODO RPM > tal comecar o logging ou tensao do inversor etc
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
    if (currentMillis[0] - previousMillis[0] > 200) {
        previousMillis[0] = currentMillis[0];
        memset(&txmsg, 0, sizeof(txmsg));
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
        can2.write(txmsg);

        memset(&txmsg, 0, sizeof(txmsg));
        txmsg.id = 0x510;
        txmsg.len = 2;
        txmsg.buf[0] = 0x01;
        can3.write(txmsg);
    }
}

/// @brief display the welcome message on the OLED display
void displayWelcome() {
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setInverseFont(1);
    u8x8.drawString(5, 0, "L.A.R.T");
    u8x8.setInverseFont(0);
    u8x8.drawString(2, 2, "Data Logger");
    u8x8.drawString(6, 3, "v1.1");
    u8x8.drawString(3, 5, "By : Perdu");
}

/// @brief Display the data logger status on the OLED display
void displayDataLoggerStatus() {
    u8x8.setFont(u8x8_font_chroma48medium8_r);

    /*
    u8x8.setInverseFont(1);
    u8x8.drawString(0, 0, "## DL Status ##");
    u8x8.setInverseFont(0);
*/
    static int previous_can1rx_status = -1;
    if (previous_can1rx_status != can1rx_status) {
        u8x8.clearLine(0);
        if (can1rx_status) {
            u8x8.drawString(0, 0, "CAN_D: OK");
        } else {
            u8x8.drawString(0, 0, "CAN_D: ERROR");
        }
        previous_can1rx_status = can1rx_status;
    }

    static int previous_can2rx_status = -1;
    if (previous_can2rx_status != can2rx_status) {
        u8x8.clearLine(1);
        if (can2rx_status)
            u8x8.drawString(0, 1, "CAN_P: OK");
        else
            u8x8.drawString(0, 1, "CAN_P: ERROR");
        previous_can2rx_status = can2rx_status;
    }

    static int previous_can3rx_status = -1;
    if (previous_can3rx_status != can3rx_status) {
        u8x8.clearLine(2);
        if (can3rx_status)
            u8x8.drawString(0, 2, "CAN_A: OK");
        else
            u8x8.drawString(0, 2, "CAN_A: ERROR");
        previous_can3rx_status = can3rx_status;
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

    // static int previous_file_num_int = -1;
    static int previous_hv500_InputVoltage = -1;
    if (previous_hv500_InputVoltage != hv500.InputVoltage) {
        u8x8.clearLine(6);
        char file_name[20] = {};
        // sprintf(file_name, "File num: %d", file_num_int);

        /*
        sprintf(file_name, "File num: %d", file_count);
        u8x8.drawString(0, 6, file_name);
        previous_file_num_int = file_num_int;
        */

        sprintf(file_name, "HV: %d", hv500.InputVoltage);
        u8x8.drawString(0, 6, file_name);
    }
}

/// @brief Start up sequence for the LEDs
void StartUpSequence() {
    const int leds[] = {LED_GPIO33, LED_GPIO34, LED_GPIO35, LED_GPIO36};
    const int numLeds = sizeof(leds) / sizeof(leds[0]);

    pinMode(LED_GPIO33, OUTPUT);
    pinMode(LED_GPIO34, OUTPUT);
    pinMode(LED_GPIO35, OUTPUT);
    pinMode(LED_GPIO36, OUTPUT);

    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < numLeds; ++i) {
            digitalWrite(leds[i], HIGH);
            delay(50);
        }

        for (int i = 0; i < numLeds; ++i) {
            digitalWrite(leds[i], LOW);
            delay(50);
        }
    }
}

/// @brief Blink the button led on the cockpit depending on the logging status
void BUTTON_LED_TASK() {
    if (logging_active) {
        digitalWrite(COCKPIT_BUTTON, HIGH);
    } else {
        FadeLed();
    }
}

/// @brief Fade the dashboard data logger led
void FadeLed() {
    static int brightness = 0;                // Current LED brightness
    static int fadeAmount = 5;                // How much to change the brightness each step
    static unsigned long previousMillis = 0;  // Stores the last time the LED was updated
    const unsigned long interval = 30;        // Interval at which to update the brightness (in milliseconds)

    unsigned long currentMillis = millis();  // Get the current time

    // Check if it's time to update the LED brightness
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;  // Save the current time

        // Update the brightness for the fade effect
        brightness += fadeAmount;

        // Reverse the fade direction if brightness reaches the limits
        if (brightness <= 0 || brightness >= 255) {
            fadeAmount = -fadeAmount;
        }

        // Set the LED brightness
        analogWrite(COCKPIT_BUTTON, brightness);
    }
}

/// @brief create a new file to log to the SD card
void createNewFile() {
    if (!SD.exists(folderName)) {
        SD.mkdir(folderName);
    }
    FILE_NAME = std::string(folderName) + "/LOG_" + std::to_string(day()) + "-" + std::to_string(month()) + "-" + std::to_string(year()) + "_" + std::to_string(hour()) + "-" + std::to_string(minute()) + "-" + std::to_string(second()) + ".asc";
    dataFile_ptr = nullptr;
    File dataFile = SD.open(FILE_NAME.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile_ptr = &dataFile;
        dataFile.close();
        dataFile_ptr = nullptr;
    } else {
        // Serial.println("Error opening file");
    }
}
/// @brief Do the can1 receive things
void Can1_things() {
    static CAN_error_t error;
    if (can1.error(error, 0)) {
        can1rx_status = false;
        digitalWrite(LED_GPIO34, LOW);  // turn off the can bus rx led
    } else {
        if (can1.read(rxmsg)) {
            decodeDataCan1(rxmsg);
            builDataString(rxmsg, 1);  // build the data string to be logged to the SD card

            digitalToggle(LED_GPIO34);  // toggle the can bus rx led
            can1rx_status = true;       // set the can1rx_status to true
        }
    }
}

/// @brief Do the can2 receive things
void Can2_things() {
    static CAN_error_t error;
    if (can2.error(error, 0)) {
        can2rx_status = false;
        digitalWrite(LED_GPIO35, LOW);  // turn off the can bus rx led
    } else {
        if (can2.read(rxmsg2)) {
            decodeDataCan2(rxmsg2);
            builDataString(rxmsg2, 2);  // build the data string to be logged to the SD card

            digitalToggle(LED_GPIO35);  // toggle the can bus rx led
            can2rx_status = true;       // set the can1rx_status to true
        }
    }
}

/// @brief Do the can3 receive things
void Can3_things() {
    static CAN_error_t error;
    if (can3.error(error, 0)) {
        can3rx_status = false;
        digitalWrite(LED_GPIO36, LOW);  // turn off the can bus rx led
    } else {
        if (can3.read(rxmsg3)) {
            decodeDataCan3(rxmsg3);
            builDataString(rxmsg3, 3);  // build the data string to be logged to the SD card

            digitalToggle(LED_GPIO36);  // toggle the can bus rx led
            can3rx_status = true;       // set the can1rx_status to true
        }
    }
}

/// @brief decode the data from the can1 bus
/// @param canmsg CAN_message_t struct
void decodeDataCan1(CAN_message_t canmsg) {
    switch (canmsg.id) {
        case CAN_HV500_ERPM_DUTY_VOLTAGE_ID:
            hv500.InputVoltage = MAP_DECODE_Actual_InputVoltage(canmsg.buf);
            break;

        default:
            break;
    }
}

/// @brief decode the data from the can2 bus
/// @param canmsg CAN_message_t struct
void decodeDataCan2(CAN_message_t canmsg) {
    switch (canmsg.id) {
        case CAN_HV500_ERPM_DUTY_VOLTAGE_ID:
            hv500.ERPM = MAP_DECODE_Actual_ERPM(canmsg.buf);
            // hv500.Duty = MAP_DECODE_Actual_Duty(canmsg.buf);
            // hv500.InputVoltage = MAP_DECODE_Actual_InputVoltage(canmsg.buf);
            break;
        case CAN_HV500_AC_DC_current_ID:
            hv500.ACCurrent = MAP_DECODE_Actual_ACCurrent(canmsg.buf);
            hv500.DCCurrent = MAP_DECODE_Actual_DCCurrent(canmsg.buf);
            break;
        case CAN_HV500_Temperatures_ID:
            hv500.TempController = MAP_DECODE_Actual_TempController(canmsg.buf);
            hv500.TempMotor = MAP_DECODE_Actual_TempMotor(canmsg.buf);
            break;
        default:
            break;
    }
}

/// @brief decode the data from the can3 bus
/// @param canmsg CAN_message_t struct
void decodeDataCan3(CAN_message_t canmsg) {
    switch (canmsg.id) {
        case 0x60:
            break;

        default:
            break;
    }
}

// asc header for file w current time

/*
date Mon Sep 30 01:52:03.848 pm 2024
base hex  timestamps absolute
internal events logged
Begin TriggerBlock Mon Sep 30 01:52:03.848 pm 2024
   0.000000 Start of measurement
End TriggerBlock
*/

void CreateHeader() {
    File dataFile = SD.open(FILE_NAME.c_str(), FILE_WRITE);
    if (dataFile) {
        // Get the current date and time
        int year = ::year();
        int month = ::month();
        int day = ::day();
        int hour = ::hour();
        int minute = ::minute();
        int second = ::second();
        int millisecond = millis() % 1000;

        // Format the date and time string
        char dateTimeStr[50];
        // TODO MELHORAR ISTO
        // day of the week

        // snprintf(dateTimeStr, sizeof(dateTimeStr), "Mon  %02d %02d:%02d:%02d.%03d %s %d", day, hourFormat12(), minute, second, millisecond, isAM() ? "am" : "pm", year);

        // Write the header information
        dataFile.print("date ");
        // dataFile.println(dateTimeStr);
        dataFile.println("Wed Oct 2 04:22:06.213 pm 2024");
        dataFile.println("base hex  timestamps absolute");
        dataFile.println("internal events logged");
        dataFile.print("Begin TriggerBlock ");
        dataFile.println("Wed Oct 2 04:22:06.213 pm 2024");  // i know this is wrong
        // dataFile.println(dateTimeStr);
        dataFile.println("  0.000000 Start of measurement");
        // dataFile.println("End TriggerBlock");

        // Close the file
        dataFile.close();
    } else {
        Serial.println("Error opening file for writing");
    }
}

void Createfooter() {
    File dataFile = SD.open(FILE_NAME.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.println("End TriggerBlock");
        dataFile.close();
    } else {
        Serial.println("Error opening file for writing");
    }
}
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%