/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"


#define DISPLAY(x) UART_write(uart, &output, x);

//UART Global Variables
char            output[64];
int             bytesToSend;

// Driver Handles -Global Variables
UART_Handle     uart;

//Driver Handles - Global Variables
I2C_Handle      i2c;

//Driver Handles - Global Variables
Timer_Handle    timer0;

volatile unsigned int TimerFlag = 0;
volatile int setPointFlagUp = 0;
volatile int setPointFlagDown = 0;

//I2C Global Variables
static const struct{
    uint8_t address;
    uint8_t resultReg;
    char *id;
    } sensors[3] = {
        {0x48, 0x0000, "11X"},
        {0x49, 0x0000, "116"},
        {0x41, 0x0001, "006"}
};

uint8_t                 txBuffer[1];
uint8_t                 rxBuffer[2];
I2C_Transaction         i2cTransaction;


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Setpoint goes up */
    setPointFlagUp = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Setpoint goes down */
    setPointFlagDown = 1;
}

/*
 *  ======== timerCallback ========
 *  Callback function for the timer interrupt
 *
 *
 *
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status){
    TimerFlag = 1;
}

void initTimer(void){
    Timer_Params        params;

    //Init the driver
    Timer_init();

    //Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;      // 1/10th of a second!
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    //Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}


void initUART(void){
    UART_Params uartParams;

    //Init the driver
    UART_init();

    //Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    //Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if(uart == NULL){
        while(1);
    }

}

// Make sure you call initUART() before calling this function
void initI2C(void){
    int8_t              i, found;
    I2C_Params          i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    //Init the Driver
    I2C_init();

    //Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    //Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if(i2c == NULL){
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while(1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf     =txBuffer;
    i2cTransaction.writeCount   =1;
    i2cTransaction.readBuf      =rxBuffer;
    i2cTransaction.readCount    =0;

    found = false;
    for (i=0; i<3; ++i){
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature Sensor not found \n\r"));
    }
}

int16_t readTemp(void){

    int             j;
    int16_t         temperature = 0;

    i2cTransaction.readCount = 2;
    if(I2C_transfer(i2c, &i2cTransaction)){

        /*
         * Extract Degrees C from received data;
         *
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be a sign extended
         */
        if(rxBuffer[0] & 0x80){
            temperature |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}
int buttonCheck(void){
    int result = 0;
    if(setPointFlagUp == 0 && setPointFlagDown == 0){

    }else if (setPointFlagUp == 1){
        result = 1;
    }else if (setPointFlagDown == 1){
        result = -1;
    }
    setPointFlagUp = 0;
    setPointFlagDown = 0;
    return result;
}

int timeCheck(int t, int m){
    int result = 0;
    if(t != 0){
        result = t % m;
    }
    return result;
}
void lightOn(void){
    GPIO_write(CONFIG_GPIO_LED_0, 1);
}

void lightOff(void){
    GPIO_write(CONFIG_GPIO_LED_0, 0);
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }


    initUART();
    initI2C();
    initTimer();

    int lastUpdate = 0;
    int timer = 0;
    int seconds = 0;
    int temperature = 0;
    int setpoint = 7;
    int heat = 0;
    /*Program States*/
    /*
     * Action States:
     * Start = Start state
     * CTIME = Check Time
     * CBTN = Check Button
     * CTMP = Check Temperature
     * UPDATE = Update
     *
     * Transitions:
     *
     * G2START: Go to Start
     * G2TIME: Go to Check Time
     * G2UPD: Go to Update
     * G2TMP: Go to Check Temperature
     * G2BTN: Go to Check Button
     *
     */
    enum THERMOSTAT{START, CTIME, CBTN, CTMP, UPDATE} thermostat;
    enum TRANSITION_GATE{G2START, G2TIME, G2UPD, G2TMP, G2BTN }transition_to_next_state;

    while(1){

        switch(thermostat){         //actions
        /*
         * This acts as the starting state, and also resets the machine
         * This state transitions to the CTIME state
         */
        case START:
            timer = 0;
            seconds = 0;
            transition_to_next_state = G2TIME;
            //DISPLAY(snprintf(output, 64, "START: transition to G2TIME \n\r"));
            break;

        /*
         * This state checks how much time has passed and transitions to the appropriate state
         * Every 200ms: go to check button (G2BTN)
         * Every 500ms: go to check temperature (G2TMP)
         * Every 1000ms: go to Update (G2UPD)
         */
        case CTIME:
            //DISPLAY(snprintf(output, 64, "timeCheck: %02d \n\r", time));
            if(timeCheck(timer, 10) == 0){
                transition_to_next_state = G2UPD;
            //    DISPLAY(snprintf(output, 64, "CTIME: transition to G2UPD \n\r"));
            }else if(timeCheck(timer, 5) == 0){
                transition_to_next_state = G2TMP;
            //    DISPLAY(snprintf(output, 64, "CTIME: transition to G2TMP \n\r"));
            }else if(timeCheck(timer, 2) == 0){
                transition_to_next_state = G2BTN;
            //    DISPLAY(snprintf(output, 64, "CTIME: transition to G2BTN \n\r"));
            }
            break;


        case CBTN:     //Every 200ms check the button flags
            setpoint = setpoint + buttonCheck();
            transition_to_next_state = G2TIME;
            //DISPLAY(snprintf(output, 64, "CBTN: transition to G2TIME \n\r"));
            break;

        case CTMP:      //Every 500ms read the temperature and update the LED
            temperature = readTemp();
            if(temperature < setpoint){
                lightOn();                  //Red Light indicates heating
                heat = 1;
            }else {
                lightOff();
                heat = 0;
            }
            transition_to_next_state = G2TIME;
            //DISPLAY(snprintf(output, 64, "CTMP: transition to G2TIME \n\r"));
            break;

        case UPDATE:    //Every 1000ms output the following to the UART
            seconds = timer * 100;
            if(lastUpdate == 1){
                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds));
                lastUpdate = 0;
            }
            transition_to_next_state = G2TIME;
            //DISPLAY(snprintf(output, 64, "UPDATE: transition to G2TIME \n\r"));
            break;

        default:
            DISPLAY(snprintf(output, 64, "ERROR: Resetting Device \n\r"));
            transition_to_next_state = G2START;
            //DISPLAY(snprintf(output, 64, "DEFAULT: transition to G2START \n\r"));
            break;
        }
        switch(thermostat){

        case START:                 //Set defaults
            if(transition_to_next_state == G2TIME){
                thermostat = CTIME;
            //    DISPLAY(snprintf(output, 64, "START \n\r"));
            }
            break;

        case CTIME:                 //Check time
            if(transition_to_next_state == G2UPD){
                thermostat = UPDATE;
                //DISPLAY(snprintf(output, 64, "CTIME: UPDATE \n\r"));
            }else if(transition_to_next_state == G2TMP){
                thermostat = CTMP;
                //DISPLAY(snprintf(output, 64, "CTIME: CTMP \n\r"));
            }else if(transition_to_next_state == G2BTN){
                thermostat = CBTN;
                //DISPLAY(snprintf(output, 64, "CTIME: CBTN \n\r"));
            }else{
                thermostat = CTIME;
                //DISPLAY(snprintf(output, 64, "CTIME: Stay \n\r"));
            }
            break;

        case CBTN:                  //Check Button
            if(transition_to_next_state == G2TIME){
                thermostat = CTIME;
            //    DISPLAY(snprintf(output, 64, "CBTN: thermostat set to CTIME \n\r"));
            }
            break;

        case CTMP:                  //Check Temp
            if(transition_to_next_state == G2TIME){
                thermostat = CTIME;
            //    DISPLAY(snprintf(output, 64, "CTMP: thermostat set to CTIME \n\r"));
            }
            break;

        case UPDATE:                //Update UART
            if(transition_to_next_state == G2TIME){
                thermostat =  CTIME;
            //    DISPLAY(snprintf(output, 64, "UPDATE: thermostat set to CTIME \n\r"));
            }
            break;

        default:                    //Unhandled Event
            thermostat = START;
            //DISPLAY(snprintf(output, 64, "DEFAULT: thermostat set to START \n\r"));

            break;

        }
        while(TimerFlag){
            TimerFlag = 0;
            ++timer;
            lastUpdate = 1;
            if(timer > 100){
                timer == 0;
            }
        }
    }

    return (NULL);
}
