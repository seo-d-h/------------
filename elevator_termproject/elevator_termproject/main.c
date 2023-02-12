#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "UARTControl.h"
#include "RFIDControl.h"

#define ADC_VREF_TYPE 0x00 // A/D 컨버터 사용 기준 전압 REFS설정
#define ADC_AVCC_TYPE 0x40 // A/D 컨버터 사용 기준 전압 AVCC설정
#define ADC_RES_TYPE 0x80 // A/D 컨버터 사용 기준 전압 RES 설정
#define ADC_2_56_TYPE 0xc0 // A/D 컨버터 사용 기준 전압 2.56 설정

#define SOL 1275 // 370Hz (2703us) 1351us

unsigned char ch;
int floor_flag = 0;
int floor_flag2 = 0;
int cds_raw =0;

//블루투스 수신
ISR(USART0_RX_vect){
	ch = UDR0;
}
//블루투스 
ISR(USART0_TX_vect){
	UDR0 = ch;
}

//택트스위치 (서보모터)
ISR(INT4_vect)
{
	if(!(PINE &(1<<PE4)))
	{
		OCR1B = 190;					// GATE Open
		_delay_ms(3000);
		OCR1B = 250;					// GATE Close
		_delay_ms(1000);
	}
}
//택트스위치 (엘리베이터 1층)
ISR(INT5_vect)
{
	if(!(PINE &(1<<PE5)))
	{
		OCR1A = 250; // 1층
	}
}

//블루투스 초기화
void Init_USART0(void){
	UCSR0A = 0x00;
	UCSR0B = (1<<RXCIE0)|(1<<TXCIE0)|(1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
	UCSR0C &= ~(1<<UMSEL0);
	
	UBRR0H = 0;
	UBRR0L = 95;
}

char STR_forLCD[20]; // LCD 출력 문자 저장 위함

//RFID 인식
int16_t RFID_loop() {
    if ( ! PICC_IsNewCardPresent()) {
        return 0;
    }
    
    if ( ! PICC_ReadCardSerial()) {
        return 1;
    }
	
    PICC_DumpToSerial(&uid);
    return 2;
}

//조도센서
unsigned int single_Read_ADC_Data_Diff(unsigned char adc_mux)
{
	unsigned int ADC_Data = 0;
	
	//if(adc_mux < 8)
	//return 0xffff; //양극 신호가 아닌 단극 mux 입력시 종료
	
	//ad 변환 채널 설정
	ADMUX &= ~(0x1f);
	ADMUX |= (adc_mux & 0x1f);
	
	ADCSRA |= (1<<ADSC); //ad변환 시작
	while(!(ADCSRA & (1<<ADIF))); //ad변환 종료 대기;
	
	ADC_Data = ADCL;
	ADC_Data |= ADCH << 8 ;
	
	return ADC_Data;
}
void ADC_Init(void)//프리러닝 모드로 초기화
{
	ADCSRA = 0x00; //adc 설정을 위한 비활성화
	ADMUX = ADC_AVCC_TYPE | (0<<ADLAR) | (0<<MUX0);
	//ADMUX = ADC_AVCC_TYPE | (0<<ADLAR);
	//REFS = 1, ADLAR = 0, MUX = 0 (ADC0 선택)
	ADCSRA = (1<<ADEN) | (0<ADFR) | (3<<ADPS0);
	//1<<ADEN : AD변환 허가 , 1<<ADFR : 프리 러닝 모드 허가
	// 3<<ADPS0 : AC변환 분주비 설정 - 8분주비
	//ADSC비트는 실제 활용에서 ADC를 읽을 때 설정하여 변환한 후
	//데이터를 읽기 위해 초기화 과정에서는 설정하지 않음
}

//외부인터럽트 (스위치)
void itrp_init(void)
{
	EIMSK = 0x30; //외부인터럽트 4,5번
	SREG |= 0x80; 
	DDRE = 0x02;
}

void cds_init(void)
{
	DDRC = 0xff;
	PORTC = 0x00;
}

void my_delay_us(unsigned int delay)
{
	int i;
	for(i=0; i<delay ; i ++)
	{
		_delay_us(1);
	}
}

void buz_sound(int time)
{
	int i, tim;
	tim = 50000/time;
	
	for(i=0;i<tim;i++)
	{
		PORTG |= (1<<PG4); //buzzer on, PORTG의 4번 핀 off
		my_delay_us(time);
		PORTG &= ~(1<<PG4);
		my_delay_us(time);
	}
	PORTG |= (1<<PG4);
}
//부저 init
void buz_init(void)
{
	DDRG |= (1<<PG4); // 부저와 연결되는 PORTG.4 를 출력으로 설정
	PORTG |= (1<<PG4); // 교육용보드는 active low여서 high로 부저 꺼줌
}
int main(void)
{
    int i=0;
    int16_t res = 0;
    char str[100];
    unsigned char rfid_ic_ver;
	
	char Message[40] = {0,};
		
    ADC_Init();
    SPI_Master_Init();
    LCD_Init(); // LCD 초기화
	cds_init();
	buz_init();
	
    //Init_USART1_IntCon(9600);

    // RFID 초기화
    RFID_Init();
    
    // RFID 제어 IC(RC522) 와의 통신 및 IC정보 확인용
    rfid_ic_ver = PCD_DumpVersionToSerial();
	
    Init_USART0();
	itrp_init();
	
	OCR1A = 250;
	OCR1B = 250;
	sei();
	
	LCD_Init(); // LCD 초기화
	LCD_Pos(0,0);
	LCD_Str("Enter Card");     // LCD 동작 확인을 위함
	
    while (1) 
    {
		DDRB|=0x60;		//PB5,6
		PORTB|=0x60;

		TCCR1A=0xA2;
		TCCR1B=0x1b;
		ICR1=4999;     //TOP

        _delay_ms(1000);
        
        // RFID_loop()는 카드가 감지되면 2를 리턴함
        res = RFID_loop();
		
        cds_raw = single_Read_ADC_Data_Diff(0b0001);
		
		sprintf(Message,"LUX : %04d ",cds_raw);
		LCD_Pos(1,0);
		LCD_Str(Message);
		
		//조도 센서 LED 제어 시작//
		if(cds_raw >= 300)
		{
			PORTC = 0xff;
		}
		else
		{
			PORTC = 0x00;
		}
		//조도센서 LED 제어 끝//
		
		//RFID 시작//
        if(res == 2){
			if(uid.uidByte[0] == 0x8C) //2층 카드 태그
			{
				OCR1A = 330;
				LCD_Pos(0,0);
				LCD_Str("2st floor           ");
				OCR1B = 190;					// GATE Open
				_delay_ms(3000);
				OCR1B = 250;					// GATE Close
				_delay_ms(1000);
			}
			else if(uid.uidByte[0] == 0x23) //3층 카드 태그
			{
				OCR1A = 400;
				LCD_Pos(0,0);
				LCD_Str("3rd floor         ");
				OCR1B = 190;					// GATE Open
				_delay_ms(3000);
				OCR1B = 250;					// GATE Close
				_delay_ms(1000);
			}
			else
			{
				//부저코드
				buz_sound(SOL);
			}
        }
		else{
			LCD_Pos(1,0);
			LCD_Str("error");
        }
		//RFID 끝 //
		
		//블루투스 통신 시작//
		if(ch == '1'){
			LCD_Pos(0,0);
			LCD_Str("1st Floor               ");
			OCR1A=250;     //-90도	1층
			ch = '0';
		}
		else if(ch == '2'){
			LCD_Pos(0,0);
			LCD_Str("2nd Floor             ");
			OCR1A=330;		// 2층
			ch = '0';
		}
		else if(ch == '3'){
			LCD_Pos(0,0);
			LCD_Str("3rd Floor             ");
			OCR1A = 400;		// 3층
			ch = '0';
		}
		else{
			ch = '0';
			LCD_Pos(0,0);
			LCD_Str("             ");
		}
		//블루투스 통신 끝//
	
    }
}

