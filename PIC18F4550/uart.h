

#ifndef USART_H_
#define	USART_H_

#include <pic18f4550.h>             /* Include PIC18F4550 header file */
#define F_CPU 8000000/64            /* Define ferquency */
#define BAUDRATE (((float)(F_CPU)/(float)baud_rate)-1)/* Define Baud value */

void USART_Init(long);              /* USART Initialization function */
void USART_TxChar(char);            /* USART character transmit function */
char USART_RxChar();                /* USART character receive function */
void USART_SendString(const char *);/* USART String transmit function */
void MSdelay(unsigned int val);     /* millisecond delay function */


#endif	/* USART_H_ */

