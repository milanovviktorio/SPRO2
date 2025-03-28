#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <time.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include <avr/eeprom.h>

void delay_ms(unsigned int ms);
void delay_hs(unsigned int hs);

int main(void) {

   uart_init();
   io_redirect();
   //i2c_init();
   //LCD_init();

   //shield board init
   DDRD = 0b11110000;
   DDRC = 0x00;
   PORTC = 0b00001111;
   DDRB |= 1 << PB5;

   DDRD &= ~(1 << DDD4);     // Clear PD4 (input mode)
   PORTD |= (1 << PORTD4);   // Enable pull-up resistor

   // Configure Timer/Counter0 (TCNT0)
   TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00); // Set prescaler to 1024

   while (1)
   {
      printf("%d\n", TCNT0);
      _delay_ms(1000);  
   }
}

void delay_ms(unsigned int ms)
{
   TCCR0A |= (1 << WGM01);
   
   OCR0A = 0xF9;

   TCCR0B |= (1 << CS01) | (1 << CS00);

   for (int i = 0; i < ms; i++)
   {
      while ((TIFR0 & (1 << OCF0A)) == 0)
      {

      } 
      TIFR0 = (1<<OCF0A);
   }
}

void delay_hs(unsigned int hs)
{
   TCCR0A |= (1 << WGM01);
   
   OCR0A = 0xF9;

   TCCR0B |= (1 << CS01) | (1 << CS00);

   for (int i = 0; i < hs*100; i++)
   {
      while ((TIFR0 & (1 << OCF0A)) == 0)
      {

      } 
      TIFR0 = (1<<OCF0A);
   }
}