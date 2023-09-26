#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL  // Set 16 MHz clock speed
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE ((( F_CPU / 16) / ( USART_BAUDRATE ))- 1)

const int _pin_dat = 7;   
const int _pin_clk = 6;
const int _pin_ena = 5;
volatile bool buttonPressed = false;

volatile int delayTime = 16;
volatile bool isPressed = false;
volatile int numberOfInterrupt = 0;

int main(void) {
  // Serial.begin(9600); //no need for serial println

  // USART Configuration
  UBRR0H = (BAUD_PRESCALE >> 8);  // Load upper 8 bits of the baud rate value
  UBRR0L = BAUD_PRESCALE;        // Load lower 8 bits of the baud rate value
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // Use 8-bit character sizes
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  // Turn on the transmission and reception circuitry

  // Initialize communication with the DS1302
  EICRA |= (1 << ISC01);  // Falling edge triggers interrupt
  EIMSK |= (1 << INT0);   // Enable external interrupt INT0
  DDRD &= ~(1 << PD2);    // Set PD2 as input
  PORTD |= (1 << PD2);    // Enable pull-up resistor on PD2
  // TIMSK0 = (1 << OCIE0A); // Turn on Timer 0 Interrupt

  //RTC init
  DDRD |= (1 << _pin_ena) | (1 << _pin_clk); // Set pin_ena and pin_clk as OUTPUT
  DDRD |= (1 << _pin_dat); // Set pin_dat to INPUT
  PORTD &= ~((1 << _pin_ena) | (1 << _pin_clk)); // Set pin_ena and pin_clk LOW
  
  //Configure for Timer 1
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);  // CTC mode with no prescaler
  OCR1A = delayTime;  // Set OCR1A for a 1 micro second delay
  TIMSK1 = 0;  //turn off Timer 1 Interrupt

  sei(); // Global interrupt enable

  while (1) {
  }
}


ISR(INT0_vect){
  // Prepare READ for "minutes" register (0x83)
    DDRD |= (1 << _pin_dat); // Set pin_dat as OUTPUT
    PORTD |= (1 << _pin_ena); // Set pin_ena HIGH
    uint8_t command = 0x83; // Address for "minutes" register
    // _writeByte(command);
    for(uint8_t b = 0; b < 8; b++)
    {
        if (command & 0x01){
            PORTD |= (1 << _pin_dat); // Set pin_dat HIGH
        }
        else{
            PORTD &= ~(1 << _pin_dat); // Set pin_dat LOW

        }
            PORTD |= (1 << _pin_clk); // Set pin_clk HIGH
            delayMicroseconds(1);

            PORTD &= ~(1 << _pin_clk); // Set pin_clk LOW
            delayMicroseconds(1);

            command >>= 1;
    }
    DDRD &= ~(1 << _pin_dat); // Set pin_dat as INPUT

    uint8_t byte = 0;
    Serial.println("Minute");
    for(uint8_t b = 0; b < 8; b++)
    {
        if (PIND & (1 << _pin_dat)) {
            // Serial.print("1");
            byte |= (0x01 << b);
        }else{
            // Serial.print("0");
        }
            PORTD |= (1 << _pin_clk); // Set pin_clk HIGH
            delayMicroseconds(1);

            PORTD &= ~(1 << _pin_clk); // Set pin_clk LOW
            delayMicroseconds(1);
    }

    Serial.println( _bcd2dec(byte & 0b01111111)); //use this to print the value
    // UDR0 = byte; //use this for osciliscopre

    PORTD &= ~(1 << _pin_ena); // Set pin_ena LOW
}

uint8_t _bcd2dec(uint8_t bcd)
{
    return ((bcd / 16 * 10) + (bcd % 16));
}
