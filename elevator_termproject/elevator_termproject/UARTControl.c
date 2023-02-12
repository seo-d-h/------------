/*
 * UARTControl.c
 *
 *  Author: lbbor
 */ 

#ifndef F_CPU
/* prevent compiler error by supplying a default */
# warning "F_CPU not defined for "UARTControl.h"
# define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include "UARTControl.h"

unsigned char RecBuff[REC_BUFF_MAX_LENGTH];
unsigned char RecBuffindex;
unsigned char RecFlg;
unsigned char RecBuff_estLength = REC_BUFF_MAX_LENGTH;

char strDedug[1000]; //

void putch_USART1(char data) {
    while( !(UCSR1A & (1<<UDRE1)));
    UDR1 = data;
}
void puts_USART1(char *str){  // 문자열 출력 루틴
    while( *str != 0){  // 문자의 마지막에는 ‘\0’이 들어가 있으므로
        putch_USART1(*str);  // ‘\0’이 나올 때까지 출력한다.
        str++;
    }
}
void sendBuff_USART1(char *str,int length){  // 문자열 출력 루틴
    while (length--){
        putch_USART1(*str);  // ‘\0’이 나올 때까지 출력한다.
        str++;
    }
}
void Init_USART1_IntCon(unsigned long int baud){
    unsigned long int UBRR_value = ((F_CPU/8/2)/baud)-1;
    // ① RXCIE1=1(수신 인터럽트 허가), RXEN0=1(수신 허가), TXEN0 = 1(송신 허가)
    UCSR1B = (1<<RXCIE1)| (1<<RXEN1) | (1 <<TXEN1);
    
    UBRR1H = (UBRR_value>>8) & 0xFF; // ; // ② 57600bps 보오레이트 설정
    UBRR1L = (UBRR_value>>0) & 0xFF; // 보오레이트 설정
    sei(); // ③ 인터럽트 동작 시작(전체 인터럽트 허가)
}

ISR(USART1_RX_vect){// 인터럽트 루틴에서의 데이터 수신
    RecBuff[RecBuffindex] = UDR1;
    RecBuffindex++;
    if(RecBuffindex > 4){  // 데이터 길이정보가 수신된 경우 패킷 길이 갱신
        RecBuff_estLength = (RecBuff[4] + 6); // 패킷길이는 데이터 길이+6
    }
    if(RecBuffindex == RecBuff_estLength){          // 수신된 데이터의 순서가 패킷 길이와 같으면 패킷 수신완료
        RecFlg = 1; // 수신 완료 플래그 활성화
    }
}

#define LOG_str(x)  puts_USART1(x)