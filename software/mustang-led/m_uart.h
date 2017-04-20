/*
 * m_uart.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Wally
 */

#ifndef M_UART_H_
#define M_UART_H_

void UARTIntHandler(void);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void uart_setup(void);



#endif /* M_UART_H_ */
