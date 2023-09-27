#include <avr/io.h>
#include <avr/interrupt.h>

volatile int currentLED = 2; 
volatile int gear = 0;
volatile bool isForward = true;
volatile int delayTime64 = 31249; //for prescaler 64 
volatile int enterInterruptTime = 0;
volatile int numberInterruptNeeded = 0;


void setup() {
  Serial.begin(9600);      
  Serial.println("Forward");   
  Serial.println("Gear 0");  

  EICRA |= (1 << ISC01) | (1 << ISC11); // Falling edge triggers interrupt

  EIMSK |= (1 << INT0); // Enable external interrupt INT0
  EIMSK |= (1 << INT1); // Enable external interrupt INT1

  DDRD &= ~(1 << PD2); // Set PD2 as input
  DDRD &= ~(1 << PD3); // Set PD3 as input

  PORTD |= (1 << PD2); // Enable pull-up resistor on PD2
  PORTD |= (1 << PD3); // Enable pull-up resistor on PD3

  DDRB |= (1<<0) | (1<<1) | (1<<2); //Set the 3 port for LED

  //Configure for Timer 1
  TCCR1A = 0; 
  TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS11); // CTC mode with prescaler 64
  OCR1A = delayTime64; // Set OCR1A for a 125 ms delay

  TIMSK1 = (1 << OCIE1A); //Turn on Timer 1 Interrupt

  sei(); // Gloval interrupt enable

  while(true){

  }
}

ISR(INT0_vect){
  if(gear+1<3){  //change to the next gear
    gear++;
  }else{
    gear = 0;
  }
  
  if (gear == 0) {
    Serial.println("Gear 0");  
    numberInterruptNeeded = 0;
  } else if(gear == 1) {
    Serial.println("Gear 1");   
    numberInterruptNeeded = 1;
  } else{
    Serial.println("Gear 2");   
    numberInterruptNeeded = 2;
  }
}

ISR(INT1_vect){
  Serial.println("Switch Direction"); 
  isForward = !isForward;
}


ISR(TIMER1_COMPA_vect) {
  if((enterInterruptTime == 0 && numberInterruptNeeded == 0) || (enterInterruptTime == 1 && numberInterruptNeeded == 1) || (enterInterruptTime == 2 && numberInterruptNeeded == 2)){
    if(isForward){
      //led go forward
      if(currentLED+1 < 3){
        currentLED++;
      }else{
        currentLED = 0;
      }

      switch (currentLED){
        case 0:
          PORTB |= (1 << 0); 
          PORTB &= ~(1 << 1); 
          PORTB &= ~(1 << 2); 
          break;
        case 1:
          PORTB &= ~(1 << 0); 
          PORTB |= (1 << 1); 
          PORTB &= ~(1 << 2); 
          break;
        case 2:
          PORTB &= ~(1 << 0); 
          PORTB &= ~(1 << 1); 
          PORTB |= (1 << 2); 
          break;
      }
    }else{
      //led go backward
      if(currentLED-1 > -1){
        currentLED--;
      }else{
        currentLED = 2;
      }

      switch (currentLED){
        case 0:
          PORTB |= (1 << 0); 
          PORTB &= ~(1 << 1); 
          PORTB &= ~(1 << 2); 
          break;
        case 1:
          PORTB &= ~(1 << 0); 
          PORTB |= (1 << 1); 
          PORTB &= ~(1 << 2); 
          break;
        case 2:
          PORTB &= ~(1 << 0); 
          PORTB &= ~(1 << 1); 
          PORTB |= (1 << 2); 
          break;
      }
    }
    enterInterruptTime = 0; //reset number of time enter interrupt

  }else if(enterInterruptTime > gear){
    enterInterruptTime = 0;
  }else{
    enterInterruptTime++;
  }
  

}
