#include <avr/io.h>
#include <avr/interrupt.h>

const int _pin_dat = 2; //Port B2
const int _pin_clk = 6; //Port D6
const int _pin_ena = 5; //Port D5

volatile int delayTime = 159;
volatile bool isPressed = false;
volatile int numberOfInterrupt = 0;

volatile uint8_t command = 0x83;
volatile uint8_t byte_data = 0;

uint8_t _bcd2dec(uint8_t bcd)  //function to convert byte to decimal
{
    return ((bcd / 16 * 10) + (bcd % 16));
}

int main(void) {
  Serial.begin(9600); //no need for serial println

  // Initialize INT0
  EICRA |= (1 << ISC01);  // falling edge triggers interrupt
  EIMSK |= (1 << INT0);   // enable external interrupt INT0
  DDRD &= ~(1 << PD2);    // set PD2 as input
  // PORTD |= (1 << PD2);    // enable pull-up resistor on PD2

  //RTC init
  DDRD |= (1 << _pin_ena) | (1 << _pin_clk); // set pin_ena and pin_clk as OUTPUT
  DDRB |= (1 << _pin_dat); // set pin_dat to OUTPUT
  PORTD &= ~((1 << _pin_ena) | (1 << _pin_clk)); // set pin_ena and pin_clk LOW
  PORTB &= ~(1 << _pin_dat);
  
  //Configure for Timer 1
  TCCR1B |= (1 << WGM12);  // CTC mode with no prescaler
  TCCR1B |= (1 << CS10);
  OCR1A = delayTime;  // set OCR1A for a 10 micro-second delay
  TIMSK1 |= (1 << OCIE1A); // turn on Timer 1 interrupt

  sei(); // Global interrupt enable

  while (1) {
  }
}

ISR(INT0_vect) {
  isPressed = true;  //set button press to true
  TCNT1 = 0;  //reset counter
  numberOfInterrupt = 0;  //reset number of interrupt
  PORTD |= (1 << _pin_ena); // set pin ena HIGH
  if (command & (1 << numberOfInterrupt)) {
    //turn on the first data pin base on the command given  
    PORTB |= (1 << _pin_dat);
  } else {
    //turn off the first data pin base on the command given  
    PORTB &= ~(1 << _pin_dat);
  }
}

ISR(TIMER1_COMPA_vect) {
//check if the button is pressed and the current interrupt number
if (isPressed && numberOfInterrupt < 32) {
    numberOfInterrupt++;  //increment the number of interrupt

    PORTD ^= (1 << _pin_clk); //Toggle on the clock pin

      if (numberOfInterrupt < 16) { //check the first 8 cycles of writing command
        if (numberOfInterrupt == 15) {
        PORTB &= ~(1 << _pin_dat);
        DDRB &= ~(1 << _pin_dat);
        }
        if(numberOfInterrupt % 2 == 0){
          if (command & (1 << numberOfInterrupt/2)){
              PORTB |= (1 << _pin_dat); // set data pin HIGH
          }
          else{
              PORTB &= ~(1 << _pin_dat); // set data pin LOW
          }
        }
      }else if(numberOfInterrupt >= 16 && numberOfInterrupt < 32){ //check the next 8 cycles of reading command
        if(numberOfInterrupt % 2 == 0){
          if (PINB & (1 << _pin_dat)){
              byte_data |= (1 << (numberOfInterrupt / 2 - 8));  //shift the read value to byte_data
          }
        }
      }



   }else if (isPressed && numberOfInterrupt >= 32) {
      PORTD &= ~(1 << _pin_ena); //set pin ena LOW
      DDRB |= (1 << _pin_dat);
      isPressed = false;  //turn off the button after the run complete

      Serial.print("*Curent Minute ");
      Serial.println(_bcd2dec(byte_data & 0b01111111));  //print the value in decimal

      PORTD &= ~(1 << _pin_clk); // set clock pin LOW
      PORTB &= ~(1 << _pin_dat); // set data pin LOW

      // TIMSK1 = 0; // Turn off Timer 1 compare match A interrupt
      byte_data = 0; //reset byte_data value

  }
}


