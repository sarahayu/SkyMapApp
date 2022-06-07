/*
 * signals.h
 *
 *  Created on: Apr 8, 2022
 *      Author: sarah
 */

#ifndef SIGNALS_H_
#define SIGNALS_H_

#ifdef    __cplusplus
extern "C" {
#endif

void configureSignals(void);

void setDCHigh(void);
void setDCLow(void);
void setRESETHigh(void);
void setRESETLow(void);
unsigned int getRemoteInputPort(void);
unsigned int getRemoteInputPin(void);

#ifdef  __cplusplus
}
#endif /* __cplusplus */


#endif /* SIGNALS_H_ */
