
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "adcpwm.h"
#include <math.h>


int main(void)
{ 
  
  uart_init();
  io_redirect();
  adc_init(); // initialize the ADC module
  printf("freak\n");

  DDRC = 0b11110000; // configure pins PC0 to PC3 as inputs
  PORTC = 0b00110000; // configure pins PC0 to PC3 to not use pullups for the ADC

  unsigned int input;

  float vcp,m,b,rs,rh,a,bb;

  unsigned char t;

  while(1)
  {
    scanf("%u", &input);
    //adc_value = adc_read(0); // Value 0-1023 representing analog voltage on pin PC0
    
    vcp=1;

    m=21/4;
    b=7/30;
    t=20;
    bb=(0.0187)*((float)t)-5.68;
    a=(1.286e+12)*exp((-0.112)*((float)t));

    printf("a : %f\n", a); 
    printf("bb : %f\n", bb); 
    printf("aaaaaaaaa : %f\n", pow((a),(1/bb)));

    printf("%f\n",(float)input);
    //rs=(vcp*m*47)/((5/1024)*((double)input)-b)-47;

    rs=(197.4*1024)/(5*((float)input)-238.93)-47;

    rh=pow((rs),(1/bb))/pow((a),(1/bb));

    printf("Result of the ADC conversion : %f\n", rs); 
    printf("Result : %f\n", rh); 
  }
}

