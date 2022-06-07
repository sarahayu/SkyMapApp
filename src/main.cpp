/*
 * Authors: Pranav Kharche & Sarah Yuniar
 */

// Driverlib includes
#include <http_utils.h>
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "uart.h"
#include "gpio.h"
#include "systick.h"
#include "pin.h"

#include "gpio_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"
#include "signals.h"

// MUST include this before drawer.h, something to do with AdaFruit messing includes up???????
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"

#include "drawer.h"
#include "other_utils.h"
#include "textrect.h"
#include "Adafruit_GFX.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define APPLICATION_VERSION     "1.4.0"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#define SPI_IF_BIT_RATE  800000
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#define FOREVER 1
#define NVIC_ST_CURRENT         0xE000E018  // SysTick Current Value Register
#define PERIOD      0x01000000  //max period



#define CH_UP       23
#define CH_DOWN     24
#define DELETE      18
#define ENTER       12
#define VOL_UP      19
#define VOL_DOWN    20

#define SHIFT       CH_UP
#define ALT         CH_DOWN
#define COLORUP     VOL_UP
#define COLORDOWN   VOL_DOWN

#define DEFAULT_SSID "Kharche"

//
// IMPORTANT: INTERRUPTS DO NOT WORK ON UARTA0!!!!!!
//
// #define UART_COMM_BASE   UARTA1_BASE
// #define UART_COMM_PERIPH PRCM_UARTA1
//#define CONSOLE         UARTA1_BASE
//#define CONSOLE_PERIPH  PRCM_UARTA1

#define BUF_SIZE 128

static void Setup();
static void DisplayBanners();
static void RequestAndDraw();
static void CheckInput();
static int  processInputRemote(void);
static void sendMessage(void);

volatile int signalTime;
volatile int intCount;
volatile int prevDataValue = -1;
volatile unsigned int counter = 0;

volatile int16_t dataRemote = 0;
volatile int dataDoneRemote = 0;

// volatile char dataUART[BUF_SIZE+1];
// volatile int dataDoneUART = 0;

char inProgressTextStr[BUF_SIZE+1];
int messageSize = 0;

char history[4*BUF_SIZE];

char currColor = '0';
int useColor = 0;

char mySSID[256];
int iTLSSockID;

#define MAX_STARS 128
#define MAX_LINES 128

Star stars[MAX_STARS];
int numStars = 0;
Line lines[MAX_LINES];
int numCLines = 0;

#define MIN_MAG 5
#define TIME_OFFSET_STEP    0.5      // in hours
#define AZIMUTH_STEP        10      // in degrees
#define ALTITUDE_STEP       10      // in degrees

MPU6050 mpu;
HMC5883L compass = HMC5883L();
uint8_t fifoBuffer[64];
Quaternion q;
float euler[3];
uint8_t accelBuffer[6];
int16_t accelData[3];
MagnetometerScaled magData;

uint8_t gOffset[] = {0x03, 0x4F, 0xFF, 0xD8, 0x00, 0x15};
uint16_t aOffsetChange[] = {0xFF69, 0xFFFA, 0x005C};
uint8_t aOffsetOriginal[6];
uint8_t aOffsetNew[6];

#define RESPONSE_MAX_LEN    5210

float az = 0, alt = 0, roll = 0;
float azCalOffset = 0;
float utc_offset = 0;

char responseBuf[RESPONSE_MAX_LEN];
char requestStr[256];

char *dataPart, *starData, *constellationData;

TextRect azAltTR, hourOffsetTR;

int frozen = 0;

int main()
{
    delay(200);

    Setup();

    DisplayBanners();

    Message("Ready to receive\n\r");

    //this is code to get a custom SSID on startup using multitap remote.
//    strcpy(inProgressTextStr, "SSID: ");
//    messageSize = 6;

//    int IDReady = 0;
//    while(IDReady == 0)
//    {
//        if (dataDoneRemote)
//        {
//            IDReady = processInputRemote();
//            dataDoneRemote = 0;
//        }
//    }
//
//    // just a newline
//    if(strlen(inProgressTextStr) == 1)
//        strcpy(mySSID, DEFAULT_SSID);
//    else
//        strcpy(mySSID, inProgressTextStr);

//    iTLSSockID = ConnectToEndpoint(mySSID);
    iTLSSockID = ConnectToEndpoint(DEFAULT_SSID);

    clearStr(inProgressTextStr);
    messageSize = 0;

    Message("Ready to send\n\r");

    fillScreen(BLACK);
    RequestAndDraw();

    while (FOREVER)
    {
        CheckInput();
    }

    Message("program exit \n\r");

    // Clear the OLED and disable CS
    fillScreen(BLACK);
    MAP_SPICSDisable(GSPI_BASE);

    return 0;
}

static void CheckInputGyro();
static void CheckInputRemote();
static void CheckInputSwitch();
static void calibrate();

static void CheckInput() {
    CheckInputGyro();
    CheckInputRemote();
    CheckInputSwitch();
}

#define RAD_TO_DEG  180.0 / M_PI

static void CheckInputGyro() {
    static int gx, gy, gz;      //variables for gravity vector. uses accelerometer data.
#define GMAX    16500
    static float mx, my, mz;    //variables for magnetometer. forms vector for magnetic north
    if (!frozen) {
        I2Cdev::readBytes(0x68, 0x3b, 6, accelBuffer);
        gx = (accelData[0] = ((accelBuffer[0]<<8) | (accelBuffer[1]&0xff)));
        gy = (accelData[1] = ((accelBuffer[2]<<8) | (accelBuffer[3]&0xff)));
        gz = (accelData[2] = ((accelBuffer[4]<<8) | (accelBuffer[5]&0xff)));

        mpu.resetFIFO();                        //FIFO overflows and gets written over if wait is too long so must clear it
        while(!(mpu.dmpPacketAvailable())){};   //wait for sensor to repopulate FIFO
        mpu.getFIFOBytes(fifoBuffer, 42);       //read FIFO and translate input.
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);

//        magData = compass.ReadScaledAxis();    //magnetometer no longer used because too much interference.
//        mx = magData.XAxis;
//        my = magData.YAxis;
//        mz = magData.ZAxis;

        //calculate altitude (or pitch) using gravity
        alt  = atan((float)gz / (float)gx) * RAD_TO_DEG;

        //calculate roll using gravity
        roll = atan((float)-gy / (float)gx) * RAD_TO_DEG;

        //get azimuth from motion processor data.
        az = euler[0] * RAD_TO_DEG;

        Report("%f, ", az);
        az += azCalOffset;          //adds calibration offset
        Report("%f \r\n", az);

        // gonna leave this here if/when we get azimuth to work
//        if (az >= 360) az -= 360;
//        else if (az < 0) az += 360;

//        Report("G\t%d\t%d\t%d\tH\t%.2f\t%.2f\t%.2f\r\n", accelData[0], accelData[1], accelData[2], mx, my, mz);
//        Report("Alt\t%.2f\t%d\t%.2f\r\n", alt, accelData[2], gz);

        RequestAndDraw();
    }
}

static void ClearUIStuff() {

    // tr_reset redraws background
    tr_reset(&azAltTR);
    tr_reset(&hourOffsetTR);
}

static void DrawUIStuff() {
    static char formatBuf[256];

    sprintf(formatBuf, "%.0fE,%.0fU,%.0f%c", az, alt, roll, 247);
    tr_outputStr(&azAltTR, formatBuf);
    Report("Out %s\r\n", formatBuf);
    sprintf(formatBuf, "%+.1fh", utc_offset);
    tr_outputStr(&hourOffsetTR, formatBuf);
}

//this checks if the input from the remote is ready and then processes it.
static void CheckInputRemote() {

    if (dataDoneRemote != 0) {
        int value = dataRemote >> 8;

        if (value == CH_UP || value == CH_DOWN
                || value == 4 || value == 6
                || value == 2 || value == 8) {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);

            switch (value) {
            case CH_UP:
                utc_offset += TIME_OFFSET_STEP;
                break;
            case CH_DOWN:
                utc_offset -= TIME_OFFSET_STEP;
                break;
            case 6:
                az += AZIMUTH_STEP;
                break;
            case 4:
                az -= AZIMUTH_STEP;
                break;
            case 2:
                alt += ALTITUDE_STEP;
                break;
            case 8:
                alt -= ALTITUDE_STEP;
                break;
            }

//            if (az >= 360) az -= 360;
//            else if (az < 0) az += 360;


            RequestAndDraw();

            GPIO_IF_LedOff(MCU_RED_LED_GPIO);

        }

        dataDoneRemote = 0;
    }
}

// check for button presses (sw2 and sw3)
// sw3: freeze-frame
// sw2: recalibrate gyroscope
static void CheckInputSwitch() {
    static int hold = 0;
    static bool hold2 = 0;

    //sw3 is pressed, toggle freeze-frame (and redraw OLED)
    if(GPIOPinRead(GPIOA1_BASE, 0x20)) {
        if (hold == 0) {
            hold = 1;

            eraseStars(stars, numStars);
            eraseConstellations(lines, numCLines);
            ClearUIStuff();

            // if frozen is to be true, draw white rect outline after starstuff (but before UI)
            if (frozen == 0) {
                frozen = 1;

                drawConstellations(lines, numCLines, 50);
                drawStars(stars, numStars, MIN_MAG);
                drawRect(0, 0, WIDTH, HEIGHT, GRAY);
            }
            // otherwise, if frozen is to be false, erase black rect outline before drawing starstuff (and UI)
            else if (frozen == 1) {
                frozen = 0;

                drawRect(0, 0, WIDTH, HEIGHT, BLACK);
                drawConstellations(lines, numCLines, 50);
                drawStars(stars, numStars, MIN_MAG);
            }

            DrawUIStuff();
        }
    }
    else
        hold = 0;

    if(GPIOPinRead(GPIOA2_BASE, 0x40))          //sw2 is pressed
    {
        if (hold2 == 0)
        {
            hold2 = 1;
            calibrate();
        }
    }
    else
        hold2 = 0;
}

static void calibrate()
{
    Message("calibrating");
    mpu.resetFIFO();
    while(!(mpu.dmpPacketAvailable())){};   // Get the Latest packet
    mpu.getFIFOBytes(fifoBuffer, 42);
    mpu.dmpGetQuaternion(&q, fifoBuffer);   // Interpret packet
    mpu.dmpGetEuler(euler, &q);
    azCalOffset = -1 * euler[0] * RAD_TO_DEG; //set new offset

    Report("new offset = %f\r\n", azCalOffset);
}

// 2-in-1 function that makes POST request and redraw OLED
static void RequestAndDraw() {
    Report("Redrawing with %.2f\r\n", utc_offset);
    clearStr(requestStr);
    clearStr(responseBuf);
    craftRequest(requestStr, az, alt, roll, MIN_MAG, utc_offset * 3600);

    if (http_post(iTLSSockID, requestStr, responseBuf, RESPONSE_MAX_LEN) == 0) {
        eraseStars(stars, numStars);
        eraseConstellations(lines, numCLines);
        ClearUIStuff();

        // if not frozen, erase the border
        if (!frozen)
            drawRect(0, 0, WIDTH, HEIGHT, BLACK);

        dataPart = getLastStrtok(responseBuf, "\r\n");
        starData = strtok(dataPart, ";");
        constellationData = strtok(NULL, ";");

        numStars = parse_stars(stars, starData, MAX_STARS);
        numCLines = parse_constellations(lines, constellationData, MAX_LINES);

        drawConstellations(lines, numCLines, 50);
        drawStars(stars, numStars, MIN_MAG);
        if (frozen)
            drawRect(0, 0, WIDTH, HEIGHT, WHITE);
        DrawUIStuff();
    }
//    craftRequest(requestStr, az, alt, roll, MIN_MAG, utc_offset * 3600);
//    http_post(iTLSSockID, requestStr, responseBuf, RESPONSE_MAX_LEN);
//
//    dataPart = getLastStrtok(responseBuf, "\r\n");
//
//    starData = strtok(dataPart, ";");
//    constellationData = strtok(NULL, ";");
//
//    numStars = parse_stars(stars, starData, MAX_STARS);
//    numCLines = parse_constellations(lines, constellationData, MAX_LINES);
//
//    drawConstellations(lines, numCLines, 50);
//    drawStars(stars, numStars, MIN_MAG);
//    DrawUIStuff();
}


//takes in the most recent button press, the previous button, and a pointer to the next available character in the message.
//returns the amount by which the size of the message has changed
int setNextChar(int num, int prevNum, char* currChar)
{
    int retVal = 0;

    if((num == COLORUP || num == COLORDOWN) && useColor)
    {
        //update the current color
        if(num == COLORUP && (++currColor) == '5' ) currColor = '0';
        if(num == COLORDOWN && (--currColor) == '/' ) currColor = '4';

        //if color change character already there, change that instead of adding new one
        if(*(currChar-2) == '/')
        {
            *(currChar-1) = currColor;
            return 0;
        }
        else
        {
            *currChar = '/';
            *(currChar+1) = currColor;
            return 2;
        }
    }
    else if(num == prevNum && (unsigned)num <= 9) //increment character if same button is pressed
    {
        currChar--;
        *currChar +=1;
    }
    else if((unsigned)num <= 9) //catch if message size should increase
    {
        retVal = 1;
    }
    else if(num == DELETE)
    {
        //special case for delete SHIFT+DELETE = delete word
        if(prevNum == SHIFT)
        {
            currChar--;
            *(currChar--) = 0; //at least delete one. this will also catch a space at the start.
            int numDeleted = -1;

            if(*currChar == '/' && *(currChar-1) == ' ')  //case where there was a space followed by color change
            {
                *(currChar--) = 0;
                *(currChar--) = 0;
                numDeleted -= 2;
            }

            while(*currChar != ' ') //keep deleting until a space is found
            {
                *(currChar--) = 0;
                numDeleted--;
            }

            if(*(currChar-2) == '/')    //catch a color change just before the space
                *(currChar-1) = currColor;
            else                        //add color back in
            {
                *(currChar+1) = '/';
                *(currChar+2) = currColor;
                numDeleted += 2;
            }

            return numDeleted;
        }
        //ALT delete clears message. if message empty, clear history.
        if(prevNum == ALT)
        {
            if(messageSize > 0) //clear the message
            {
                clearStr(inProgressTextStr);
                currColor = '0';
                messageSize = 0;
            }
            return 0;
        }

        //if the character that would be deleted is a color change, the character before the color change should be deleted instead.
        if(*(currChar-2) == '/')
        {
            // however, if color code is the only character in the string, keep it
            if (messageSize == 2)
            {
                return 0;
            }

            *(currChar-3) = '/';
            *(currChar-2) = *(currChar-1);
            *(currChar-1) = 0;
            //its possible that there will be two color changes in a row. The previous color change must be deleted
            if(*(currChar-5) == '/')
            {
                *(currChar-4) = *(currChar-2);
                *(currChar-2) = 0;
                *(currChar-3) = 0;
                return -3;
            }
            else return -1;
        }
        //standard delete replaces the most recent character with null character.
        *(currChar-1) = 0;
        return -1;
    }

    char currCharVal = *currChar; //dereference first so that it doesnt repeatedly do that

    //updating the current character
    if(num == 1)
    {
        if((currCharVal == ':'))
            *currChar = '0';
        else if(currCharVal == 0)
            *currChar = '1';
    }
    else if(num == 2)
    {
        if((currCharVal == 0 && prevNum == SHIFT)|| currCharVal == 'D')
            *currChar = 'A';
        else if(currCharVal == 0 || currCharVal == 'd')
            *currChar = 'a';
    }
    else if(num == 3)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'G')
            *currChar = 'D';
        else if(currCharVal == 0 || currCharVal == 'g')
            *currChar = 'd';
    }
    else if(num == 4)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'J')
            *currChar = 'G';
        else if(currCharVal == 0 || currCharVal == 'j')
            *currChar = 'g';
    }
    else if(num == 5)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'M')
            *currChar = 'J';
        else if(currCharVal == 0 || currCharVal == 'm')
            *currChar = 'j';
    }
    else if(num == 6)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'P')
            *currChar = 'M';
        else if(currCharVal == 0 || currCharVal == 'p')
            *currChar = 'm';
    }
    else if(num == 7)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'T')
            *currChar = 'P';
        else if(currCharVal == 0 || currCharVal == 't')
            *currChar = 'p';
    }
    else if(num == 8)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == 'W')
            *currChar = 'T';
        else if(currCharVal == 0 || currCharVal == 'w')
            *currChar = 't';
    }
    else if(num == 9)
    {
        if((currCharVal == 0 && prevNum == SHIFT) || currCharVal == '[')
            *currChar = 'W';
        else if(currCharVal == 0 || currCharVal == '{')
            *currChar = 'w';
    }
    else if(num == 0 && (currCharVal == 0 || currCharVal == '/'))
    {
        *currChar = ' ';
    }
    else if(num == ENTER)
    {
        *currChar = '\n';
        return 1;
    }
    return retVal;
}

static void BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//defines for time lengths
#define GREATEST_REPEAT 25 //old value 24.2
#define SMALLEST_REPEAT 20 //old value 21.2
#define START_DATA      12
#define FENCE_DATA      4.5
#define ONE             2
#define ZERO            1
#define EPS             0.05
#define REPEAT_EPS      0.55

static void RemoteSignalInterruptHandler(void) {
    unsigned long ulStatus;
    static int count = 0;
    static int skip = 0;

    ulStatus = MAP_GPIOIntStatus (getRemoteInputPort(), true);
    MAP_GPIOIntClear(getRemoteInputPort(), ulStatus);        // clear interrupts on GPIOA0
    signalTime = SysTickValueGet();

    float sinceLastFall = (float)(PERIOD - signalTime) / 80000;

    if (sinceLastFall > GREATEST_REPEAT + REPEAT_EPS)
    {
        dataRemote = 0;
        count = 0;
    }
    else if (sinceLastFall > SMALLEST_REPEAT - EPS)
    {
        dataRemote = 0;
        count = 0;
        skip = 1;
    }
    else if (sinceLastFall > START_DATA - EPS)
    {
        dataRemote = 0;
        count = 0;
    }
    else if (sinceLastFall > FENCE_DATA - EPS)
    {
    }
    else if (sinceLastFall > ONE - EPS)
    {
        dataRemote = dataRemote >> 1;
        dataRemote = dataRemote | 0x8000;
        count++;
    }
    else if (sinceLastFall > ZERO - EPS)
    {
        dataRemote = dataRemote >> 1;
        dataRemote = dataRemote & 0x7fff;
        count++;
    }

    if (count == 16)
    {
        if (skip == 0) dataDoneRemote = 1;
        skip = 0;
    }

    HWREG(NVIC_ST_CURRENT) = 1; //reset systic timer and its counter variable.
    counter = 0;
    intCount++;
}

static void TickerHandler()
{
    counter++;
    if (counter >= 5)
    {
        counter = 0;
        if(prevDataValue <= 9) prevDataValue = -1;
    }
}

//#define UART_COMM_BASE CONSOLE
//#define UART_COMM_PERIPH CONSOLE_PERIPH

/*
static void UARTReceiveHandler()
{
    unsigned long ulStatus = UARTIntStatus(UART_COMM_BASE, true);
    UARTIntClear(UART_COMM_BASE, ulStatus);        // clear interrupts on UARTA1 (?)

    if (ulStatus & UART_INT_RX)
    {
        while (UARTCharsAvail(UART_COMM_BASE))
        {
            char c = UARTCharGet(UART_COMM_BASE);

            int len = strlen(dataUART);

            if (c == '\n' || c == '\r')
            {
                dataDoneUART = 1;
                return;
            }

            // make sure we're not going out of bounds
            if (len < BUF_SIZE)
                dataUART[len] = c;
            // if message too long, full send
            else
                dataDoneUART = 1;
        }
    }
}
*/

static void SetupUART()
{
    InitTerm();
    ClearTerm();

    // UARTConfigSetExpClk(UART_COMM_BASE,PRCMPeripheralClockGet(UART_COMM_PERIPH),
    //                 UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    //                  UART_CONFIG_PAR_NONE));

    // unsigned long ulStatus = UARTIntStatus(UART_COMM_BASE, true);
    // UARTIntClear(UART_COMM_BASE, ulStatus);        // clear interrupts on UARTA1
    // UARTIntRegister(UART_COMM_BASE, UARTReceiveHandler);
    // UARTFIFODisable(UART_COMM_BASE);
    // UARTIntEnable(UART_COMM_BASE, UART_INT_RX);
}

static void SetupSPI()
{
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);
}

static void SetupInt()
{
    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(getRemoteInputPort(), RemoteSignalInterruptHandler);

    //
    // Configure falling edge interrupts from IR receiver
    //
    MAP_GPIOIntTypeSet(getRemoteInputPort(), getRemoteInputPin(), GPIO_FALLING_EDGE);

    unsigned int ulStatus;
    ulStatus = MAP_GPIOIntStatus (getRemoteInputPort(), false);
    MAP_GPIOIntClear(getRemoteInputPort(), ulStatus);            // clear interrupts on GPIOA1

    // Enable interrupts
    MAP_GPIOIntEnable(getRemoteInputPort(), getRemoteInputPin());
}

static void SetupTicker()
{
    SysTickPeriodSet(PERIOD);
//    SysTickIntEnable();
//    SysTickIntRegister(TickerHandler);
    SysTickEnable();
}

static void SetupGyro()
{
    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    Message("initializing");

    uint8_t resetData[2] = {0x6b, 0x80};
    I2C_IF_Write(0x68, resetData, 2, 1);    //reset sensor
    delay(1);
    mpu.initialize();                       //initialize things like clock source and sensitivity
    delay(1);
    bool status = mpu.testConnection();     //test the connection using the WHO_AM_I register which returns the I2C address
    if(status) Message("MPU success\r\n");
    else Message("MPU error\r\n");

    compass.SetScale(1.3);                                  //initialize magnetometer
    compass.SetMeasurementMode(Measurement_Continuous);

    delay(1);
    Message("dmp");
    status = mpu.dmpInitialize();                   //initialize motion processor
    if(status==0) Message("DMP success \r\n");      //the program for the processor must be flashed on startup
    else Message("DMP error\r\n");                  //it apparently doesnt store that in nonvolotile memory
    delay(5);

    /* set offsets to calibrate the sensors. finding the offsets can be automated but was not
     * The gyroscope offsets are default zero but the accel offsets have a factory set default so they must be added rather than set.
     */
    I2Cdev::writeBytes(0x68, 0x13, 6, gOffset);
    I2Cdev::readBytes(0x68, 0x06, 6, aOffsetOriginal);
    for(int i = 0; i <3;i++)
    {
        aOffsetChange[i] += ((aOffsetOriginal[2*i]<<8) | (aOffsetOriginal[2*i+1]&0xff));
        aOffsetNew[2*i] = (aOffsetChange[i] >> 8) & 0xFF;
        aOffsetNew[2*i+1] = aOffsetChange[i] & 0xFF;
    }
    I2Cdev::writeBytes(0x68, 0x06, 6, aOffsetNew);

    mpu.setDMPEnabled(true);    //enable motion processor
    delay(1);
    mpu.resetFIFO();
    calibrate();
}

static void SetupUIStuff() {
    tr_initBG(&azAltTR, 0, HEIGHT - GLYPH_HEIGHT, WIDTH, HEIGHT, WHITE, BLACK);
    tr_initBG(&hourOffsetTR, WIDTH - GLYPH_WIDTH * 5, HEIGHT - GLYPH_HEIGHT, WIDTH, HEIGHT, WHITE, BLACK);
}

static void Setup()
{
    BoardInit();
    PinMuxConfig();

    configureSignals();

    SetupUART();
    SetupSPI();
    SetupInt();
    SetupTicker();
    SetupGyro();
    SetupUIStuff();

    MAP_SPICSEnable(GSPI_BASE);

    Adafruit_Init();
//    Report("report\r\n");
//    Message("message");
//    incomingText = malloc(sizeof(TextRect));
//    tr_init(incomingText, 20, HEIGHT / 2 + 6, WIDTH, HEIGHT, WHITE);
}

static void DisplayBanners()
{
    // for PuTTY
    Report("\n\n\n\r");
    Report("\t *************************************************\n\r");
    Report("\t        CC3200 %s Application       \n\r", "AWS messaging");
    Report("\t *************************************************\n\r");
    Message("\n\n\r");

    // for OLED
    fillScreen(BLACK);

    TextRect adastraTR;
    int bannerWidth = 4 * WIDTH / 5 - 4 * GLYPH_WIDTH, bannerHeight = GLYPH_HEIGHT * 2;
    tr_init(&adastraTR,
            WIDTH / 2 - bannerWidth / 2, HEIGHT / 2 - bannerHeight / 2,
            WIDTH / 2 + bannerWidth / 2, HEIGHT / 2 + bannerHeight / 2,
            WHITE);
    tr_outputStr(&adastraTR, "Ad astra per aspera.");
}

static int processInputRemote(void)
{
    int dataValue, dataAddr;

    dataValue = dataRemote >> 8;
    dataAddr = dataRemote & 0xff; //never actually use this information but whatever

    Report("Received dataRemote %d at %#x    previous value %d\n\r", dataValue, dataAddr, prevDataValue);

    if(messageSize < BUF_SIZE-2) //minus two because null character is needed and two characters can be entered in one button
    {
        messageSize += setNextChar(dataValue, prevDataValue, inProgressTextStr+messageSize);
        if(messageSize < 0) messageSize = 0;
    }
    Report("Message: %s %d\n\r", inProgressTextStr, messageSize);

    prevDataValue = dataValue;

    if(dataValue == ENTER)
        return 1;
    else
        return 0;
}

static void sendMessage(void)
{
    strcat(history, inProgressTextStr);

    char messageToPost[256];
//    strcpy(messageToPost, JSON_START);
//    strncat(messageToPost, inProgressTextStr, messageSize-1);
//    strcat(messageToPost, JSON_END);

    //TODO: send message
//    http_post(iTLSSockID, messageToPost);

    clearStr(inProgressTextStr);
    if(useColor)
    {
        inProgressTextStr[0] = '/';
        inProgressTextStr[1] = currColor;
        messageSize = 2;
    }
    else
    {
        messageSize = 0;
    }
    Message("message sent");

}
