/*
 * signals.c
 *
 *  Created on: Apr 8, 2022
 *      Author: sarah
 */


#include "signals.h"
#include "gpio_if.h"

#define DC          0
#define RESET       1
#define REMOTE_INPUT      2
#define COUNT       3

#define GPIO_DC         3
#define GPIO_RESET      28
#define GPIO_REMOTE_INPUT     7

unsigned int GPIO_MAP[COUNT] =
{
     GPIO_DC,
     GPIO_RESET,
     GPIO_REMOTE_INPUT
};

unsigned int PORT_MAP[COUNT];
unsigned char PIN_MAP[COUNT];


void configureSignals(void)
{
    GPIO_IF_GetPortNPin(GPIO_MAP[DC],
                        &PORT_MAP[DC],
                        &PIN_MAP[DC]);
    GPIO_IF_GetPortNPin(GPIO_MAP[RESET],
                        &PORT_MAP[RESET],
                        &PIN_MAP[RESET]);

    GPIO_IF_GetPortNPin(GPIO_MAP[REMOTE_INPUT],
                        &PORT_MAP[REMOTE_INPUT],
                        &PIN_MAP[REMOTE_INPUT]);
}

void setDCHigh(void)
{
    GPIO_IF_Set(GPIO_MAP[DC], PORT_MAP[DC], PIN_MAP[DC], 1);
}

void setDCLow(void)
{
    GPIO_IF_Set(GPIO_MAP[DC], PORT_MAP[DC], PIN_MAP[DC], 0);
}

void setRESETHigh(void)
{
    GPIO_IF_Set(GPIO_MAP[RESET], PORT_MAP[RESET], PIN_MAP[RESET], 1);
}

void setRESETLow(void)
{
    GPIO_IF_Set(GPIO_MAP[RESET], PORT_MAP[RESET], PIN_MAP[RESET], 0);
}

unsigned int getRemoteInputPort(void)
{
    return PORT_MAP[REMOTE_INPUT];
}

unsigned int getRemoteInputPin(void)
{
    return PIN_MAP[REMOTE_INPUT];
}
