#include <avr/io.h>
#include <avr/interrupt.h>

volatile int delayTime = 6249;
volatile bool isPressed = false;
volatile int numberOfInterrupt = 0;

void setup() {
  Serial.begin(9600);  // Testing

  EICRA |= (1 << ISC11);  // Falling edge triggers interrupt
  EIMSK |= (1 << INT1);   // Enable external interrupt INT1
  DDRD &= ~(1 << PD3);    // Set PD3 as input
  PORTD |= (1 << PD3);    // Enable pull-up resistor on PD3

  DDRB |= (1 << 0) | (1 << 1);  //Set the 2 port for LED

  //Configure for Timer 1
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12);  // CTC mode with prescaler 256
  OCR1A = delayTime;                    // Set OCR1A for a 100 ms delay

  TIMSK1 = 0;  //turn off Timer 1 Interrupt
  // TIMSK1 = (1 << OCIE1A); //Turn on Timer 1 Interrupt

  sei();  // Gloval interrupt enable

  while (true) {
  }
}

ISR(INT1_vect) {
  Serial.println("---------Button Pressed---------");
  isPressed = true;
  TIMSK1 |= (1 << OCIE1A);
  TCNT1=0;
}


ISR(TIMER1_COMPA_vect) {
  if (isPressed) {
    if (numberOfInterrupt == 0) {
      Serial.print("Turn on Green Led at interrupt ");  // Testing
      Serial.println(numberOfInterrupt);
      PORTB |= (1 << 0);
    } else if (numberOfInterrupt == 33) {
      Serial.print("Turn off Green Led at interrupt ");  // Testing
      Serial.println(numberOfInterrupt);
      PORTB &= ~(1 << 0);
      isPressed = false;
      TIMSK1 = 0;  //turn off Timer
    }

    if (numberOfInterrupt >= 1 && numberOfInterrupt < 33) {
      Serial.print("GPIO2 interrupt number ");  // Testing
      Serial.println(numberOfInterrupt);
      PORTB ^= (1 << 1);
    }

    if (numberOfInterrupt + 1 <= 33) {
      numberOfInterrupt++;
    } else {
      numberOfInterrupt = 0;
    }
  }
}
