#include <avr/io.h>
#include <avr/interrupt.h>

// #ifndef F_CPU
// #define F_CPU 16000000UL  // Set 16 MHz clock speed
// #endif

// #define USART_BAUDRATE 9600
// #define BAUD_PRESCALE ((( F_CPU / 16) / ( USART_BAUDRATE ))- 1)

const int _pin_dat = 2;   
const int _pin_clk = 6;
const int _pin_ena = 5;
volatile bool buttonPressed = false;

volatile int delayTime = 159; //10 micro second
volatile bool isPressed = false;
volatile int numberOfInterrupt = 0;

volatile uint8_t command = 0x83;
volatile uint8_t byte_data = 0;
volatile int counterRead = 0;

uint8_t _bcd2dec(uint8_t bcd)
{
    return ((bcd / 16 * 10) + (bcd % 16));
}

int main(void) {
  Serial.begin(9600); //no need for serial println

  // // USART Configuration
  // UBRR0H = (BAUD_PRESCALE >> 8);  // Load upper 8 bits of the baud rate value
  // UBRR0L = BAUD_PRESCALE;        // Load lower 8 bits of the baud rate value
  // UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // Use 8-bit character sizes
  // UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  // Turn on the transmission and reception circuitry

  // Initialize communication with the DS1302
  EICRA |= (1 << ISC01);  // Falling edge triggers interrupt
  EIMSK |= (1 << INT0);   // Enable external interrupt INT0
  DDRD &= ~(1 << PD2);    // Set PD2 as input
  // PORTD |= (1 << PD2);    // Enable pull-up resistor on PD2

  //RTC init
  DDRD |= (1 << _pin_ena) | (1 << _pin_clk); // Set pin_ena and pin_clk as OUTPUT
  DDRB |= (1 << _pin_dat); // Set pin_dat to OUTPUT
  PORTD &= ~((1 << _pin_ena) | (1 << _pin_clk)); // Set pin_ena and pin_clk LOW
  PORTB &= ~(1 << _pin_dat);
  
  //Configure for Timer 1
  TCCR1B |= (1 << WGM12);  // CTC mode with no prescaler
  TCCR1B |= (1 << CS10);
  OCR1A = delayTime;  // Set OCR1A for a 1 micro second delay
  TIMSK1 |= (1 << OCIE1A); // Turn on Timer 1 compare match A interrupt

  sei(); // Global interrupt enable

  while (1) {
  }
}

ISR(INT0_vect) {
  isPressed = true;
  TCNT1 = 0;  //reset counter
  numberOfInterrupt = 0;
  PORTD |= (1 << _pin_ena); // Set pin_ena HIGH
  if (command & (1 << numberOfInterrupt)) {
    PORTB |= (1 << _pin_dat);
  } else {
    PORTB &= ~(1 << _pin_dat);
  }
}

ISR(TIMER1_COMPA_vect) {

if (isPressed && numberOfInterrupt < 32) {
    numberOfInterrupt++;

    PORTD ^= (1 << _pin_clk);


      if (numberOfInterrupt < 16) {
        if (numberOfInterrupt == 15) {
        PORTB &= ~(1 << _pin_dat);
        DDRB &= ~(1 << _pin_dat);
        }
        if(numberOfInterrupt % 2 == 0){
          if (command & (1 << numberOfInterrupt/2)){
              PORTB |= (1 << _pin_dat); // Set pin_dat HIGH
          }
          else{
              PORTB &= ~(1 << _pin_dat); // Set pin_dat LOW
          }
        }
      }else if(numberOfInterrupt >= 16 && numberOfInterrupt < 32){
        if(numberOfInterrupt % 2 == 0){
          if (PINB & (1 << _pin_dat)){
              byte_data |= (1 << (numberOfInterrupt / 2 - 8));
          }
        }
      }



   }else if (isPressed && numberOfInterrupt >= 32) {
      PORTD &= ~(1 << _pin_ena);
      DDRB |= (1 << _pin_dat);
      isPressed = false;

      Serial.print("*Curent Minute ");  // Testing
      Serial.println(_bcd2dec(byte_data & 0b01111111));

      PORTD &= ~(1 << _pin_clk); // Set pin_ena LOW
      PORTB &= ~(1 << _pin_dat);

      // TIMSK1 = 0; // Turn on Timer 1 compare match A interrupt
      byte_data = 0;

  }
}


