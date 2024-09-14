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
// #include <USBHost_t36.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//#include "cansart.h"
#include "SerialTransfer.h"
#include "Watchdog_t4.h"

#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

#define CANSART 1

#define HV_PrechargeVoltage 280

SerialTransfer myTransfer;

#if CANSART

int k = 0;

struct frame
{
uint8_t ID ;
uint8_t DATA1 ;
uint8_t DATA2 ;
uint8_t DATA3 ;
uint8_t DATA4 ;
uint8_t DATA5 ;
uint8_t DATA6 ;
uint8_t DATA7 ;
uint8_t DATA8 ;
uint8_t LENGHT ;
};

//frame frame11;
//frame frame20;
//frame frame30;
//frame frame60;
//frame frame121;

struct STRUCT {
  frame frames11;
  frame frames20;
  frame frames30;
  frame frames60;
  frame frames121;
} os;

//HardwareSerial& serialPort = Serial5;
#endif

// #define rx7Pin 28
// #define tx7Pin 29

// SoftwareSerial MySerial7(rx7Pin, tx7Pin);  // RX, TX

//________________________________________________________________________________________________
//__________________________________Defines_________________________________________________________
//________________________________________________________________________________________________
#define DataLogger_CanId 0x200
#define XANATO_PIN 29

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

#define Button_LED 37
#define Button_Cockpit 28

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
void builDataString(CAN_message_t msg, uint8_t can_identifier);                // Build the data string to be logged to the SD card
void displayWelcome(void);                                                     // Display the welcome message on the OLED
void displayDataLoggerStatus(void);                                            // Display the data logger status on the OLED
void Can1_things(void);                                                        // Do the can1 receive things
void Can2_things(void);                                                        // Do the can2 receive things
void Can3_things(void);                                                        // Do the can3 receive things
void StartUpSequence(void);                                                    // LEDs startup sequence
void BUTTON_LED_TASK(void);                                                    // Blink the button led
void createNewFile(void);                                                      // Create a new file to log to the SD card
void FadeLed(void);                                                            // Fade the led

WDT_T4<WDT1> wdt;                                                 // Watchdog timer config
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(SCL, SDA, U8X8_PIN_NONE);  // OLED contructor
//_________________________________________________________________________________________________
//__________________________________Variables______________________________________________________
//_________________________________________________________________________________________________

volatile bool logging_active = false;    // If the logging is active or not
volatile bool DataLoggerActive = false;  // If the data logger is active or not
char FILE_NAME[50] = {};                 // Name of the file to be logged to the SD card
const char* folderName = "logs";         // Folder name to store the logs
String dataString = "";                  // String to be logged to the SD card

int file_count = 0;                      // Count how many files are in the SD card
int file_num_int = 0;                    // Keep track of the datalog.txt file number
int file_num_plus_one = 0;               // Keep track of the datalog.txt file number
File* dataFile_ptr;                      // Pointer to the datafile to be closed by the interrupt
bool SD_card_status = false;             // If the SD card is present and initialized

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

uint16_t HV = 0;          // High voltage
long long HV500_RPM = 0;  // rpm from inverter

//______Telemetria_______
float CAN_Bus_Data[50];
IntervalTimer TelemetryTimer;
void sendTelemetry(void);
void updateData(void);
void SetFrames(void);

void wdtCallback() {
    Serial.println("FEED THE DOG SOON, OR RESET!");
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
    setSyncProvider(getTeensy3Time);  // Set the Time library to use Teensy 3.0's RTC to keep time
#ifdef __Telemetria_ON__
    TelemetryTimer.begin(sendTelemetry, 20000);  // interval timer for telemetry
#endif

    pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin as an output
    pinMode(XANATO_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the built in led
    digitalWrite(XANATO_PIN, LOW);    // turn on the built in led

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
//    setCANSART_Driver(serialPort, (unsigned long)115200);
    os.frames11.ID = 11;
    os.frames20.ID = 20;
    os.frames30.ID = 30;
    os.frames60.ID = 60;
    os.frames121.ID = 121;
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
            delay(2000);
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

    // count how many files are in the SD card
    /*
    File root = SD.open("/");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break;
        }
        file_count++;
        entry.close();
    }
    root.close();
    */
    
    // ###################################################################################################

    Serial.println("█▀▀ ▄▀█ █▄░█   █▄▄ █░█ █▀   █▀▄ ▄▀█ ▀█▀ ▄▀█   █░░ █▀█ █▀▀ █▀▀ █▀▀ █▀█");
    Serial.println("█▄▄ █▀█ █░▀█   █▄█ █▄█ ▄█   █▄▀ █▀█ ░█░ █▀█   █▄▄ █▄█ █▄█ █▄█ ██▄ █▀▄");

    /*############### Record Button ################*/
    pinMode(BUTTON_PIN, INPUT);
    pinMode(Button_LED, OUTPUT);
    digitalWrite(Button_LED, LOW);

    pinMode(OPAMP_PIN, INPUT_PULLDOWN);
    pinMode(Button_Cockpit, INPUT_PULLUP);
    /*############### Update RTC by serial port ################*/
    RTC_update_by_serial();

    attachInterrupt(  // attach interrupt to opamp protection pin
        digitalPinToInterrupt(OPAMP_PIN), []() {
            if (dataFile_ptr != nullptr){
                dataFile_ptr->close();
            }
        },
        RISING);

#ifdef ENABLE_INTERRUPT
/*
    attachInterrupt(  // attach interrupt to the button pin
        digitalPinToInterrupt(BUTTON_PIN), []() {
            // debounce
            if (millis() - previousMillis[2] < 300) {
                return;
            }
            previousMillis[2] = millis();
            if (dataFile_ptr != nullptr)
                dataFile_ptr->close();

            logging_active = !logging_active;
            Serial.println(logging_active ? "Logging started" : "Logging stopped");

            // is first time logging active?create the file name with the current time
            // static bool firstime = true;
            if (logging_active) {
                // firstime = false;
                sprintf(FILE_NAME, "LOG_%02d-%02d-%02d_%02d-%02d-%02d.csv", day(), month(), year(), hour(), minute(), second());
            }
        },
        RISING);
*/
#else
    logging_active = true;
#endif

#ifdef ENABLE_EXTERNAL_BUTTON
    attachInterrupt(  // attach interrupt to the button pin
        digitalPinToInterrupt(Button_Cockpit), []() {
            // debounce
            if (millis() - previousMillis[2] < 20) {
            } else {
                if (dataFile_ptr != nullptr) {
                    dataFile_ptr->close();
                }
                logging_active = !logging_active;

                if (logging_active) {
                    sprintf(FILE_NAME, "LOG_%02d-%02d-%02d_%02d-%02d-%02d.csv", day(), month(), year(), hour(), minute(), second());
                }
            }
            previousMillis[2] = millis();
        },
        RISING);
#else
    logging_active = true;
#endif

    /*##############################################*/

    /*#################### CAN INIT ################*/
    can1.begin();
    can1.setBaudRate(1000000);
    can2.begin();
    can2.setBaudRate(1000000);
    can3.begin();
    can3.setBaudRate(1000000);
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
    // logging_active = false;
    
    Serial5.begin(115200);
    myTransfer.begin(Serial5);
}

void loop() {
    wdt.feed();  // Feed the whatchdog timer

    MCU_heartbeat();  // Blink the built in led at 3.3Hz

    Can1_things();  // Do the can1 receive things
    Can2_things();  // Do the can2 receive things
    Can3_things();  // Do the can3 receive things

    BUTTON_LED_TASK(); // Blink the button led on the cockpit

    sendDLstatus();  // Send the data logger status to the CAN bus

    if((HV > 60) && !logging_active){
        createNewFile();
        logging_active = true; // start logging
    }
    
    if (millis() - previousMillis[4] > 15) { // log to sd card every 15ms
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

#if CANSART

    updateData();
   // Serial5.println("ola");
   /*
    if (Serial5.available()) {
        char buffer[100] = {};
        Serial5.readBytesUntil('\n', buffer, 100);


        Serial.println(buffer);
    }*/

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
    //fade the led
    
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

    File dataFile = SD.open(FILE_NAME, FILE_WRITE);
    dataFile_ptr = &dataFile;

    if (dataFile) {
        dataFile.print(dataString);
        dataFile.close();
        dataFile_ptr = nullptr;
    } else {

        Serial.println("error opening LOG file");
    }
    dataString = "";
}

/**
 * @brief Build the data string to be logged to the SD card
 */
/*
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
*/
void builDataString(CAN_message_t msg, uint8_t can_identifier) {

    // HH:MM:SS.mmm;
    dataString += String(hour());
    dataString += ":";
    dataString += String(minute());
    dataString += ":";
    dataString += String(second());
    dataString += ".";
    dataString += String(milliseconds_calculation());
    dataString += csvSuffixer(can_identifier, DEC);
    dataString += csvSuffixer(msg.id, HEX);
    dataString += csvSuffixer(msg.len, DEC);
    for (int i = 0; i < msg.len; i++) {
        dataString += csvSuffixer(msg.buf[i], DEC);
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
        if (HV > HV_PrechargeVoltage) {
            txmsg.buf[7] = 1;
        } else {
            txmsg.buf[7] = 0;
        }

        can1.write(txmsg);
        can2.write(txmsg);

        memset(&txmsg, 0, sizeof(txmsg));
        txmsg.id = 0x510;
        txmsg.len = 2;
        int32_t HV500_RPM_10 = HV500_RPM / 10;
        txmsg.buf[0] = HV500_RPM_10 >> 8;
        txmsg.buf[1] = HV500_RPM_10;

        can3.write(txmsg);
    }
}

void displayWelcome() {
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setInverseFont(1);
    u8x8.drawString(5, 0, "L.A.R.T");
    u8x8.setInverseFont(0);
    u8x8.drawString(2, 2, "Data Logger");
    u8x8.drawString(6, 3, "v1.1");
    u8x8.drawString(3, 5, "By : Perdu");
}

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

    static int previous_file_num_int = -1;
    if (previous_file_num_int != file_num_int) {
        u8x8.clearLine(6);
        char file_name[20] = {};
        // sprintf(file_name, "File num: %d", file_num_int);
        /*
        sprintf(file_name, "File num: %d", file_count);
        u8x8.drawString(0, 6, file_name);
        previous_file_num_int = file_num_int;*/

        sprintf(file_name, "RPM: %d", HV500_RPM);
        u8x8.drawString(0, 6, file_name);
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
    static CAN_error_t error;
    if (can1.error(error, 0)) {
        // Serial.println("CAN1 ERROR");
        can1rx_status = false;
        digitalWrite(LED_GPIO34, LOW);  // turn off the can bus rx led
    } else {
        if (can1.read(rxmsg)) {
            switch (rxmsg.id) {
                case 0x60:
#if CANSART
/*
                    frames60.DATA1 = rxmsg.buf[0];
                    frames60.DATA2 = rxmsg.buf[1];
                    frames60.DATA3 = rxmsg.buf[2];
                    frames60.DATA4 = rxmsg.buf[3];
                    frames60.DATA5 = rxmsg.buf[4];
                    frames60.DATA6 = rxmsg.buf[5];
                    frames60.DATA7 = rxmsg.buf[6];
                    frames60.DATA8 = rxmsg.buf[7];*/
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
/*
                    frames61.DATA1 = rxmsg.buf[0];
                    frames61.DATA2 = rxmsg.buf[1];
                    frames61.DATA3 = rxmsg.buf[2];
                    frames61.DATA4 = rxmsg.buf[3];
                    frames61.DATA5 = rxmsg.buf[4];
                    frames61.DATA6 = rxmsg.buf[5];
                    frames61.DATA7 = rxmsg.buf[6];
                    frames61.DATA8 = rxmsg.buf[7];*/
#endif
                    break;
                case 0x62:
                    break;
                case 0x80:
#if CANSART
/*
                    frames80.DATA1 = rxmsg.buf[0];
                    frames80.DATA2 = rxmsg.buf[1];
                    frames80.DATA3 = rxmsg.buf[2];
                    frames80.DATA4 = rxmsg.buf[3];
                    frames80.DATA5 = rxmsg.buf[4];
                    frames80.DATA6 = rxmsg.buf[5];
                    frames80.DATA7 = rxmsg.buf[6];
                    frames80.DATA8 = rxmsg.buf[7];*/
#endif
                    break;
                case 0x200:
#if CANSART
/*
                    frames120.DATA1 = rxmsg.buf[0];
                    frames120.DATA2 = rxmsg.buf[1];
                    frames120.DATA3 = rxmsg.buf[2];
                    frames120.DATA4 = rxmsg.buf[3];
                    frames120.DATA5 = rxmsg.buf[4];
                    frames120.DATA6 = rxmsg.buf[5];
                    frames120.DATA7 = rxmsg.buf[6];
                    frames120.DATA8 = rxmsg.buf[7];*/
#endif
                    break;
                case 0x202:
                    // receiveMarker(rxmsg, 0x202, 0);
                default:
                    // add ids to filter
                    break;
            }

            digitalToggle(LED_GPIO34);  // toggle the can bus rx led
            builDataString(rxmsg, 1);   // build the data string to be logged to the SD card
            can1rx_status = true;       // set the can1rx_status to true
            // Serial.print(rxmsg.id);     // print the data string to the serial port
        }
    }
}

void Can2_things() {
    static CAN_error_t error;
    if (can2.error(error, 0)) {
        can2rx_status = false;
        digitalWrite(LED_GPIO35, LOW);  // turn off the can bus rx led
    } else {
        if (can2.read(rxmsg2)) {
            digitalToggle(LED_GPIO35);  // toggle the can bus rx led
            builDataString(rxmsg2, 2);  // build the data string to be logged to the SD card
            can2rx_status = true;       // set the can1rx_status to true
            // Serial.print(rxmsg2.id);  // print the data string to the serial port
            if (rxmsg2.id == 0x14) {
                HV = rxmsg2.buf[6] << 8 | rxmsg2.buf[7];
                if (HV > HV_PrechargeVoltage) {
                    
                    digitalWrite(XANATO_PIN, HIGH);
                } else {
                    digitalWrite(XANATO_PIN, LOW);
                }
            }
            if (rxmsg2.id == 0x14) {
                HV500_RPM = rxmsg2.buf[0] << 24 | rxmsg2.buf[1] << 16 | rxmsg2.buf[2] << 8 | rxmsg2.buf[3];
                // HV500_RPM = rxmsg2.buf[2];
                // HV500_RPM = HV500_RPM * (-1);
            }
        }
    }
}

void Can3_things() {
    static CAN_error_t error;
    if (can3.error(error, 0)) {
        can3rx_status = false;
        digitalWrite(LED_GPIO36, LOW);  // turn off the can bus rx led
    } else {
        if (can3.read(rxmsg3)) {
            digitalToggle(LED_GPIO36);  // toggle the can bus rx led
            builDataString(rxmsg3, 3);  // build the data string to be logged to the SD card
            can3rx_status = true;       // set the can1rx_status to true
            // Serial.print(rxmsg3.id);  // print the data string to the serial port
        }
    }
}

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

void BUTTON_LED_TASK(void) {

    if (logging_active) {
        digitalWrite(Button_LED, HIGH);
    } else {
        FadeLed();
    }
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// CANSART VARS

uint16_t RPM = 0;
uint16_t InputVoltage = 0;
uint16_t ACCurrent = 0;
uint16_t DCCurrent = 0;

uint8_t INVTemp = 0;
uint8_t MotorTemp = 0;
bool DriveEnable = false;
uint8_t VCUState = 0;
uint8_t TCUState = 0;
uint8_t DLState = 0;
uint8_t ACUState = 0;
uint8_t PCState = 0;

#if CANSART

void updateData(void) {
    SetFrames();
    RPM = 300;

    os.frames20.DATA1 = RPM >> 8;
    os.frames20.DATA2 = RPM;
    os.frames20.DATA3 = InputVoltage >> 8;
    os.frames20.DATA4 = InputVoltage;
    os.frames20.DATA5 = ACCurrent >> 8;
    os.frames20.DATA6 = ACCurrent;
    os.frames20.DATA7 = DCCurrent >> 8;
    os.frames20.DATA8 = DCCurrent;

    INVTemp = 10;

    os.frames60.DATA1 = INVTemp;
    os.frames60.DATA2 = MotorTemp;
    os.frames60.DATA3 = DriveEnable;
    os.frames60.DATA4 = VCUState;
    os.frames60.DATA5 = TCUState;
    os.frames60.DATA6 = DLState;
    os.frames60.DATA7 = ACUState;
    os.frames60.DATA8 = PCState;

    os.frames30.DATA1 = k;
    os.frames30.DATA2 = k;
    os.frames30.DATA3 = k;
    os.frames30.DATA4 = k;
    os.frames30.DATA5 = k;
    os.frames30.DATA6 = k;
    os.frames30.DATA7 = k;
    os.frames30.DATA8 = k;
    k++;
    if(k>250){
        k=0;
    }
    //updateDB(&frames11);
    //updateDB(&frames20);
    //updateDB(&frames60);
    //updateDB(&frames61);
    //updateDB(&frames121);
    myTransfer.sendDatum(os);
}
void SetFrames(){
    os.frames11.ID = 11;
    os.frames20.ID = 20;
    os.frames30.ID = 30;
    os.frames60.ID = 60;
    os.frames121.ID = 121;
}

#endif

/// @brief Fade the dashboard data logger led
/// @param  
void FadeLed(void) {

  static int brightness = 0;                   // Current LED brightness
  static int fadeAmount = 5;                   // How much to change the brightness each step
  static unsigned long previousMillis = 0;     // Stores the last time the LED was updated
  const unsigned long interval = 30;           // Interval at which to update the brightness (in milliseconds)

  unsigned long currentMillis = millis();      // Get the current time

  // Check if it's time to update the LED brightness
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the current time

    // Update the brightness for the fade effect
    brightness += fadeAmount;

    // Reverse the fade direction if brightness reaches the limits
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }

    // Set the LED brightness
    analogWrite(Button_LED, brightness);
  }
}

void createNewFile(void) {

    // Create the folder if it doesn't exist
    if (!SD.exists(folderName)) {
        SD.mkdir(folderName);
    }

    // Format the file name with the folder path
    sprintf(FILE_NAME, "%s/LOG_%02d-%02d-%02d_%02d-%02d-%02d.csv", folderName, day(), month(), year(), hour(), minute(), second());

    // Open the file in the folder
    File dataFile = SD.open(FILE_NAME, FILE_WRITE);
    if (dataFile) {
        dataFile_ptr = &dataFile;
        dataFile.close();
    } else {
        Serial.println("Error opening file");
    }
}
