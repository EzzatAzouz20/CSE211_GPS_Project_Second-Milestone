#include "stdint.h"
#include "C:/Keil/EE319Kware/inc/tm4c123gh6pm.h"
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