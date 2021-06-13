#include "stdint.h"
#include "tm4c123gh6pm.h"
#define RED 0x02
#include "math.h"
#include "C:/Keil/EE319Kware/SysTick_4C123/SysTick.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#include <ctype.h>
#define PI  3.14159265358 
#define R   6371
void SystemInit(){}
// Function to initialize port F 
void init(){
SYSCTL_RCGCGPIO_R	|= 0x20;
while ((SYSCTL_PRGPIO_R&0x20)==0){};
GPIO_PORTF_LOCK_R = 0x4C4F434B;
GPIO_PORTF_CR_R = 0x1F;
GPIO_PORTF_DIR_R = 0x0E;
GPIO_PORTF_DEN_R = 0x1F;
GPIO_PORTF_AMSEL_R = 0;
GPIO_PORTF_AFSEL_R = 0;
GPIO_PORTF_PCTL_R = 0;
GPIO_PORTF_PUR_R = 0x11;
}




//LCD Delay and Command and Data
void LCD_command(unsigned char command)
{
	GPIO_PORTA_DATA_R =0x00;
  GPIO_PORTB_DATA_R =command;
  GPIO_PORTA_DATA_R |=0x80;
  GPIO_PORTA_DATA_R =0x00;
}

void systic_init(void){
    NVIC_ST_CTRL_R=0;
    NVIC_ST_RELOAD_R=0x00FFFFFF;
    NVIC_ST_CURRENT_R=0;
    NVIC_ST_CTRL_R = 0x05;
}
	void delay(uint32_t time){
        NVIC_ST_RELOAD_R = time-1;
      NVIC_ST_CURRENT_R = 0;
      while((NVIC_ST_CTRL_R&0X00010000)==0){};
}
	void delay_micro(uint32_t time){
 unsigned long i;
    for(i=0;i<time;i++)
    delay(80);
}
void delay_milli(uint32_t time){
 unsigned long i;
    for(i=0;i<time;i++)
    delay_micro(1000);

}

void LCD_data(unsigned char data)
{
	GPIO_PORTA_DATA_R =0x20;
  GPIO_PORTB_DATA_R =data;
  GPIO_PORTA_DATA_R |=0x80;
  GPIO_PORTA_DATA_R =0x00;
}


void LCD_init(){
SYSCTL_RCGCGPIO_R|= 0x03;//Enable port A & port B
while ((SYSCTL_PRGPIO_R&0x03)==0){};
	
	//INITIALIZATION OF PORT A
GPIO_PORTA_DIR_R |= 0xE0;//set A5,A6 & A7 pins to be output
GPIO_PORTA_DEN_R |= 0xE0;//set A5,A6 & A7 pins to be digital
GPIO_PORTA_AMSEL_R =0x00;
GPIO_PORTA_AFSEL_R =0x00;
GPIO_PORTA_PCTL_R =0x00;
GPIO_PORTA_PUR_R =0x00;
	
//INITIALIZATION OF PORT B
  GPIO_PORTB_DIR_R = 0xFF;//set all pins to be output
  GPIO_PORTB_DEN_R = 0xFF;//set all pins to be gigital
  GPIO_PORTB_AMSEL_R =0x00;
  GPIO_PORTB_AFSEL_R =0x00;
  GPIO_PORTB_PCTL_R =0x00;
  GPIO_PORTB_PUR_R =0x00;
	LCD_command(0x0F);//turn on display
	LCD_command(0x38);//2 lines (8 bits data)
	LCD_command(0x01);//ciear display
}

//  calculating the taken distance
float total_distance(float * total_distance ,float lat1, float long1, float lat2, float long2)
{
	 
	double x1, x2, delta_x, delta_y;
  double a, c, d;
	x1 = lat1 * (PI / 180);
  x2 = lat2 * (PI / 180);
  delta_x = (lat2 - lat1) * (PI / 180);
  delta_y = (long2 - long1) * (PI / 180);

  a = (sin(delta_x / 2) * sin(delta_x / 2)) + cos(x1) * cos(x2) * sin(delta_y / 2) * sin(delta_y / 2);
  c = 2 * atan2(sqrt(a), sqrt(1-a));
  d = R * c;
	*total_distance = *total_distance + d; 
	
	return d;
}
  void UART_Init(void){
	SYSCTL_RCGCUART_R |= 0x00000002;	// Enable uart1
	SYSCTL_RCGCGPIO_R |= 0x00000004;	//Enable portc

	UART1_CTL_R &= ~0x00000001; //DISABLE UART CONTROL
	
	//Neo-6 Baud rate = 9600;
	UART1_IBRD_R = 104;
	UART1_FBRD_R = 11;
	
	UART1_LCRH_R = 0x00000070;	//no parity one stop & 2 fifos
	UART1_CTL_R |= 0x00000001;	//ENABLE UART AGAIN
	
	
	GPIO_PORTC_AFSEL_R |= 0x30;		
	GPIO_PORTC_DEN_R |= 0x30;//DIGITAL ENABLE of c4-c5
	
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0xFF00FFFF) + 0x00220000; 
	GPIO_PORTC_AMSEL_R &= ~0x30;	

}
void UART0_Init(void)
{
SYSCTL_RCGCUART_R |= 1;
SYSCTL_RCGCGPIO_R |= 1;
UART0_CTL_R = 0;
UART0_IBRD_R = 104;
UART0_FBRD_R = 11;
UART0_CC_R = 0;
UART0_LCRH_R = 0x60;
UART0_CTL_R |= 0x301;
GPIO_PORTA_AFSEL_R |= 0x03 ;
GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0XFFFFF00)|0x00000011;
GPIO_PORTA_DEN_R |= 0x03;
}



char UART_Read(void){
	while ((UART1_FR_R&0x0010) != 0);    //busy waiting 
	
	return (UART1_DR_R&0xFF);  
}
uint8_t uart0_available(void)
{
	return((UART0_FR_R&UART_FR_RXFE)==UART_FR_RXFE)? 0:1;
}
float read(void)
{
	while(uart0_available()!=1);
	return (float)(UART0_DR_R&0xFF);
}
void UART0_Write(uint8_t data){
	while((UART0_FR_R & UART_FR_TXFF) !=0);
	UART0_DR_R = data;
}
int data_line (char * data) {

    char d;
    int i;
    i=0;
    d = UART_Read();
    while(d != '$') {
        d = UART_Read();
    }
    while (d != '*'){
        data[i]= d;
        d = UART_Read();
        i = i+1;
    }
    if(data[1] == 'G' && data[2] == 'P' && data[3] == 'G' && data[4] == 'G' && data[5] == 'A') {
        return i+1;
    }
    else {
        return -1;
    }
}
void swap(char * LA1, char * LO1, char * LA2, char * LO2)
{
	uint8_t i;
	for( i=0; i<9; i++)
	{
		LA1[i] = LA2[i];
	}
	for(i=0; i<10; i++)
	{
		LO1[i] = LO2[i];
	}
}





uint32_t  parsing (char * Gpsdata, char * latitude, char * longitude, int size)
{	int lat_start, lat_end, lon_start ,lon_end;
    uint32_t i;
		uint32_t j;
		uint32_t k;
		uint32_t a;
		uint32_t b;
		uint32_t flag1;
	uint32_t flag2;
	uint32_t comma_counter;
	flag1=0;
	flag2=0;
	comma_counter=0;
    
    for(i = 0; i<size; i++) {
			
        if(Gpsdata[i] == ',') {
            comma_counter++;
            if(comma_counter == 2) {
                lat_start = i+1;
            }
            if(comma_counter == 3) {
                lat_end = i-1;
            }
            if(comma_counter == 4) {
                lon_start = i+1;
            }
            if(comma_counter == 5) {
                lon_end = i-1;							
            }
        }
    }
    a = 0;
    for(j=lat_start; j<=lat_end; j++) {
        latitude[a] =(char) (Gpsdata[j]);
        a++;
			flag1=1;
    }
    b = 0;
    for(k=lon_start; k<=lon_end; k++) {
        longitude[b] = (char)(Gpsdata[k]);
        b++;
			flag2=1;
    }
		
		 
		if(flag1&&flag2)
				return 1;
		else
		    return 0;
}
void dec_to_str (char* str, uint32_t val,uint32_t digits)
{   
  uint32_t i=1u;

  for(; i<=digits; i++)
  {
    str[digits-i] = (char)((val % 10u) + '0');
    val/=10u;
  }
  str[i-1u] = '\0'; // assuming you want null terminated strings?
}
uint32_t number_digits(uint32_t num)
{
    uint32_t count;
    count=0;
    while(num!=0)
    {
        num=num/10;
        count++;     
    }
    return count;
}
void LCD_DISTANCE(uint32_t number)
{ char change[10];
	uint32_t y;	
		if(number==0)
	{LCD_data('0');
	delay_milli(1);
	return;}
	dec_to_str(change,number,number_digits(number));
		for(y=0;y<number_digits(number);y++)
		{
			LCD_data(change[y]);
			delay_milli(1);
		}
}

double atof_m(const char *s){
    int i;
	int sign;
	double value;
	double power;
	int powersign;
	int power2;
    for(i = 0; isspace(s[i]); ++i);
        

    
    sign = (s[i] == '-')? -1 : 1; 

    if(s[i] == '-' || s[i] == '+'){
        ++i;
    }

    
    for(value = 0.0; isdigit(s[i]); ++i){
        value = value * 10.0 + (s[i] - '0');
    }

    if(s[i] == '.'){
        ++i;
    }

    
    for(power = 1.0; isdigit(s[i]); ++i){
        value = value * 10.0 + (s[i] - '0');
        power *= 10.0;
    }

    if(s[i] == 'e' || s[i] == 'E'){
        ++i;
    }
    else{
        return sign * value/power;
    }

     //The sign following the E
    powersign = (s[i] == '-')? -1 : 1;

    if(s[i] == '-' || s[i] == '+'){
        ++i;
    }

     //The number following the E
    for(power2 = 0; isdigit(s[i]); ++i){
        power2 = power2 * 10 + (s[i] - '0');
    }

    if(powersign == -1){
        while(power2 != 0){
            power *= 10;
            --power2;
        }
    }
    else{
        while(power2 != 0){
            power /= 10;
            --power2;
        }
    }

    return sign * value/power;
}
void LCD_STRING(char *str)
{
  uint32_t size;
	//uint8_t up;
	uint32_t y;
	
	size=strlen(str);
	//up=1;
	for(y=0;y<size;y++)
		{	delay_milli(10);
			LCD_data(str[y]);
			
		}
}
int main(){
	init();
	uint32_t size;
	float lat1_no, lon1_no, lat2_no, lon2_no;
	float total_dist;
	char data[90];
	char lat1[9];
	char lon1[10];
	char lat2[9];
	char lon2[10];
	char out_test[10] = {'1','2','3','.','2','4','6','7'};
	uint8_t i=0;

	size = 0;
	total_dist = 0;
	UART_Init();

	systic_init();
	//portF_Init();
	//switch_inter();
	LCD_init();
	UART0_Init();
		while (i<10){
			UART0_Write(out_test[i]);
			i=i+1;
		}
	LCD_command(0x01);//clear
	LCD_command(0x80);// start from 1st line
	delay_milli(100);
	
	delay_milli(10);
		LCD_DISTANCE(lon1_no);
	do {
		size = data_line(data);

	} while (size == -1);


	

	if (parsing(data, lat1, lon1, size))
	{
		while (1)
		{
			void Turn_Led(void);
	  LCD_command(0x01);
		LCD_command(0x80);
			//uint32_t size;
			size = 0;
			lat1_no = atof(lat1);
			lon1_no = atof(lon1);
			do {
				size = data_line(data);
			} while (size == -1);
			if (parsing(data, lat2, lon2, size))
			{
				lat2_no = atof(lat2);
				lon2_no = atof(lon2);
				total_distance(&total_dist, lat1_no, lon1_no, lat2_no, lon2_no);
				//print dist
				LCD_command(0x01);//clears
				LCD_command(0x80);// start from 1st line
				LCD_STRING("dist=");
				delay_milli(100);
				LCD_DISTANCE(total_dist);
				delay_milli(100);

				swap(lat1, lon1, lat2, lon2);
				//delay_milli(100);
			}

		}
	}
	}