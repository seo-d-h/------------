/*
 * UARTControl.h
 *
 *  Author: lbbor
 */ 


#ifndef UARTCONTROL_H_
#define UARTCONTROL_H_

#define REC_BUFF_MAX_LENGTH 100
#define LOG_str(x)  puts_USART1(x)

extern char strDedug[1000];


void putch_USART1(char data);
void puts_USART1(char *str);
void sendBuff_USART1(char *str,int length);
void Init_USART1_IntCon(unsigned long int baud);

#endif /* UARTCONTROL_H_ */