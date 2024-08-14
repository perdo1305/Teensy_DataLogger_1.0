# Microcontroller LIB

Hello Everyone,

Instructions:

1º-  Download "CANSART_API" folder;

2º-  Paste the folder in your project and do the includes!

# Be aware:

## STM32:
    - You might have to do include on porject settings due to the working principle of STM environment, if you are familiar with stm32 and including libraries you are good to go, otherwise you need to do:
    1º - Click on "Project" Tab;
    2º - Click "Properties";
    3º - Click "C/C++ Build";
    4º - Click "Settings";
    5º - On "MCU G++ Compiler" click "Include paths";
    6º - On "Include paths" box, top right, click "Add...";
    7º - Give your path to "inc" folder from library, something like: "...\CANSART_API\inc";
    8º - Click on "Apply" then on "Apply and Close";
    9º - Include "cansart.h" on your project;

## ARDUINO
    - Most of the time is very straight forward like including the files in your project folder and include it on the main.ino or main.cpp file;

3º - On cansart_db.h you have to:

    1º - Set architecture;
    
    2º - Add your IDs structures like the one that are pre-inserted;


4º - On "main.cpp" or "yourfile.cpp" you have to:

    1º - ```#include "cansart.h"```;
    
    2º - Instanciate your used IDs like frame10 frames10;
    
    3º - Call 2 functions on startup:
    
        1º - cansart_init_Frames(); Where you will initialize your frames by setting the ID;
        
        2º - cansart_init(USART Driver,BaudRate); Where you can set the driver that you will use and the BaudRate, for some chips you might have a 3rd (RX Pin) and 4th parameter(TX Pin) to set pins (ESP32 for example);
   
    4º - On runtime you will call cansart_updateDB(&frames10) to update the database, be aware that you need to call this funciton for each used ID!; 


5º - Check The examples to help you out!


6º - Thanks...!
