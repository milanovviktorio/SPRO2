
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
//#include "adcpwm.h"
#include <math.h>


int main(void)
{ 
  
  uart_init();
  io_redirect();
  //adc_init(); // initialize the ADC module
  printf("freak\n");

  DDRC = 0b11110000; // configure pins PC0 to PC3 as inputs
  PORTC = 0b00110000; // configure pins PC0 to PC3 to not use pullups for the ADC

  unsigned int inputHumidity;

  float capVolt,scalingFactor,offset,resExp,rh,opampFactor,z,capOffset,capFactor;

  unsigned char tempSet;

  while(1)
  {
    scanf("%u", &inputHumidity);
    //adc_value = adc_read(0); // Value 0-1023 representing analog voltage on pin PC0
    
    opampFactor = 4.75;
    offset = 0.2325;
    tempSet = 55;
    capOffset=1.28;
    capFactor=-6e-04;
    capVolt=capFactor*inputHumidity+capOffset;
    resExp=(0.0187)*(tempSet)-5.68;
    scalingFactor = (1.286e+12)*exp((-0.112)*(tempSet));
  
    //z=(vcp*m*47)/((5/1024)*((double)input)-b)-47;

    printf("uno : %f\n", (capVolt*opampFactor*1024*47)); 
    printf("dos : %f\n", ((4.8)*((float)inputHumidity)-offset*1024)); 
    printf("tre : %f\n", (capVolt*opampFactor*1024*47)/((4.8)*((float)inputHumidity)-offset*1024)); 
  
    z=(capVolt*opampFactor*1024*47)/((4.8)*((float)inputHumidity)-offset*1024)-47;
  
    rh=pow((z),(1/resExp))/pow((scalingFactor),(1/resExp));
  

    printf("Result of the ADC conversion : %f\n", z); 
    printf("Result : %f\n", rh); 
  }
}

