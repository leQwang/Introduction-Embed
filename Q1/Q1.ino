#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int var = 0;
volatile bool isHighFreq = true;

void setup() {
  Serial.begin(9600);      // Testing 
  EICRA |= (1 << ISC01); // Falling edge triggers interrupt
  EIMSK |= (1 << INT0); // Enable INT0
  DDRD &= ~(1 << PD2); // Set PD2 as input
  PORTD |= (1 << PD2); // Enable pull-up resistor on PD2
  DDRB |= (1<<5); 

  //Configure for Timer 0
  TCCR0A = (1 << WGM01); //Set CTC mode
  TCCR0B = (1 << CS00) | (1 << CS01); // Prescaling by 64
  OCR0A = 249; // Set OCR0A for 500Hz frequency

  TIMSK0 = (1 << OCIE0A); //Turn on Timer 0 Interrupt


  //Configure for Timer 1
  TCCR1A = 0; 
  TCCR1B = (1 << WGM12) | (1 << CS12); // CTC mode with prescaler 256
  OCR1A = 6249; // Set OCR1A for a 100 ms delay

  TIMSK1 = 0; // TIMSK1 = (1 << OCIE1A); //Turn off Timer 1 Interrupt


  sei(); // Gloval interrupt enable

  while(true){

  }
}

ISR(INT0_vect){
  isHighFreq = !isHighFreq; // Toggle between high and low frequencies
  if (isHighFreq) {
    Serial.println("500Hz");
    TIMSK0 = (1 << OCIE0A); 
    TIMSK1 = 0; 

  } else {
    Serial.println("100Hz");
    // Configure Timer 1 for 5Hz
    TIMSK0 = 0; 
    TIMSK1 = (1 << OCIE1A);
  }
}

ISR(TIMER0_COMPA_vect) {
  PORTB ^= (1 << 5); // toggle port B5
  
}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << 5); // toggle port B5
}

