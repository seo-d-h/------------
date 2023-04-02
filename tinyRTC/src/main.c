#define F_CPU 14745600UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "tinyRTC.h"

#define ADC_VREF_TYPE 0x00 // A/D 컨버터 사용 기준 전압 REFS설정
#define ADC_AVCC_TYPE 0x40 // A/D 컨버터 사용 기준 전압 AVCC설정
#define ADC_RES_TYPE 0x80 // A/D 컨버터 사용 기준 전압 RES 설정
#define ADC_2_56_TYPE 0xc0 // A/D 컨버터 사용 기준 전압 2.56 설정

int cds_data;
unsigned int ADC_Data=0;
//differential adc 결과 읽어오는 함수
//  ad 변환 값
void ADC_Init(void){
	ADCSRA = 0x00;
	ADMUX=((ADC_AVCC_TYPE) | (0<<ADLAR) | (0<<MUX0));
	ADCSRA = (1<<ADEN) | (1<<ADFR) | (3<<ADPS0);
}

unsigned int Read_ADC_Data_Diff(unsigned char adc_mux){
	unsigned int ADC_Data = 0;
	
	if(adc_mux < 8)
		return 0xFFFF;
	
	ADMUX &= ~(0x1F);
	ADMUX |= (adc_mux & 0x1F);
	
	ADCSRA |= (1<<ADSC);
	
	while(!(ADCSRA & (1<<ADIF)));
	
	ADC_Data = ADCL;
	ADC_Data |= ADCH << 8;
	
	return ADC_Data;
}

int main(void)
{
	int adcRaw=0;
	float adcmilliVoltage = 0;
	float Celsius = 0;
	char c_Message[40]={0,};
	DDRB = 0xff;
	DDRE = 0x0f;
	
	PORTB = 0xff;
	
	ADC_Init();

	
	char Message[40] = {0,};
	uart0_init();
	
	tinyRTC_init();

	tinyRTC_setup(00, 30, 16, 5, 22, 12, 22);	// 초, 분, 시, 요일, 일, 월, 년
	tinyRTC_set_date();
	LCD_Init();

    while (1) 
    {
		tinyRTC_read_date();

		adcRaw = Read_ADC_Data_Diff(0b1101);
		adcmilliVoltage = ((((float)adcRaw * 5000)/512)/10);
		Celsius = adcmilliVoltage / 10;
		
		sprintf(Message,"Temp(deg):%2.2f",Celsius);
		LCD_Pos(1,0);
		LCD_Str(Message);
		_delay_ms(100);

    }
	return 0;
}
