/********************************************
 *
 *  Name: Vincent Onggoputra
 *  Email: onggoput@usc.edu
 *  Section: Friday 11 a.m
 *  Assignment: Final Project
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"

volatile uint32_t timerVal = 0;
volatile char start = 1;
volatile char stop = 1;
volatile char done = 0;
// encoder variables
volatile uint16_t speedLimit = 1;
volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;
volatile uint8_t a, b;
// buzzer variables
volatile char isBuzzing = 0;
volatile uint16_t buzzerCount = 0;
const uint16_t buzzerMaxCount = 500;
// serial vairables
volatile char rxbuffer[6];  // Including '\0'
volatile int buffer_count = 0;
volatile char data_started = 0;
volatile char data_valid = 0;
volatile uint16_t local_speed;

void timer0_init(void);
void timer1_init(void);
void timer2_init(void);
void tx_speed(int speed);
void display_speed();
int main(void){

    // initializing code
    lcd_init();
    timer0_init();
    timer1_init();
    timer2_init();

    // init encoder
    PORTD |= ((1 << 2) | (1 << 3));
    PCMSK2 |= ((1 << PCINT18) | (1 << PCINT19));
    PCICR |= (1 << PCIE2);
    // end init

    // speed trap interrupt init
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT12) | (1 << PCINT13); // enable pin change interrupts
    DDRB |= (1 << 5);
    sei();
    // end init

    // serial init
    DDRB |= (1 << 4);
    UBRR0 = (16e6 / 16 / 9600 - 1); 
    UCSR0B |= (1 << TXEN0 | 1 << RXEN0); // enable RX and TX
    UCSR0C = (3 << UCSZ00); // ASYNC, one stop bit, 8 data bits
    PORTD &= ~(1 << 4);
    UCSR0B |= (1 << RXCIE0); // enable reciever interrupts
    // end serial

    // RGB init
    DDRC |= (1 << 2) | (1 << 3); // 2 is blue 3 is red
    PORTC |= ((1 << 2) | (1 << 3));

    // end rgb init
    // splash screen
    lcd_writecommand(0x01);
    lcd_moveto(0,0);
    lcd_stringout("Final Project");
    lcd_moveto(1,0);
    lcd_stringout("Vincent");
    _delay_ms(1000);
    lcd_writecommand(0x01);

    // init phototransistors
    PORTC |= (1 << 4) | (1 << 5); // enable pullups for phototransistor
    uint8_t savedLimit = eeprom_read_byte((void*) 200);
    if(savedLimit <= 99 && savedLimit >= 0){
        speedLimit = savedLimit;
    }
    while(1){
        _delay_ms(100);
            if(changed){
            changed = 0;
            lcd_moveto(1,0);
            char buf3[10];
            if(speedLimit < 10){
                snprintf(buf3, 10, "Max = %u ", speedLimit);
            }else{
                snprintf(buf3, 10, "Max = %u", speedLimit);
            }
            
            lcd_stringout(buf3);
            eeprom_update_byte((void *) 200, speedLimit);
        }
        if(done){
            char buf[17];
            snprintf(buf, 17, "%ld ms = ", timerVal);
            lcd_moveto(0,0);
            lcd_stringout(buf);
            
            uint32_t speed = 40000 / timerVal;
            uint16_t speed_ones = speed / 10;
            uint8_t speed_tenths = speed % 10;
            char buf2[17];
            snprintf(buf2, 17, "   %u.%1u",speed_ones, speed_tenths);
            lcd_stringout(buf2);
            done = 0;


            // buzzer logic
            if(speed_ones > speedLimit && !isBuzzing){
                TCCR0B |= (1 << CS02); // start timer
                isBuzzing = 1;
            }else if(speed_ones <= speedLimit && isBuzzing){
                TCCR0B &= ~(1 << CS02);
                PORTC &= ~(1 << PC1);
                isBuzzing = 0;
            }

            // servo logic
            uint32_t temp = 35 - speed_ones * 23 / 100;
            OCR2A = temp % 35;

            // transmit speed
            local_speed = speed_ones * 10 + speed_tenths;
            tx_speed(local_speed);
            if(!data_valid){
                PORTC |= ((1 << 2) | (1 << 3));
            }
        }
        
    }
}

// speed controller
void timer1_init(void)
{
    TCCR1B |= (1 << WGM12);     // Set for CTC mode using OCR1A for the modulus
    TIMSK1 |= (1 << OCIE1A);    // Enable CTC interrupt
    OCR1A = 62500;              // Set the speedLimiter modulus correctly
}

ISR(PCINT1_vect){
    if((PINC & (1 << 4)) == 0){ 
        TCNT1 = 0;
        TCCR1B |= (1 << CS12) | (1 << CS10); // set prescalar
        PORTB |= (1 << 5);
    }

    if((PINC & (1 << 5)) == 0){
        timerVal = TCNT1;
        timerVal = (timerVal * 40) / 625; 
        done = 1;
        TCCR1B &= ~((1 << CS12) | (1 << CS10)); // clear prescalar
        PORTB &= ~(1 << 5);   
    }
}

ISR(TIMER1_COMPA_vect){
    lcd_writecommand(0x01);
    lcd_moveto(0,0);
    lcd_stringout("Timing expired");
    PORTB &= ~(1 << 5);
}
// end speed controller

// rotary encoder
ISR(PCINT2_vect)
{
	
	uint8_t pinc_val = PIND;
    a = (pinc_val & (1 << PD2)) != 0;
    b = (pinc_val & (1 << PD3)) != 0;

	if (old_state == 0) {

	    // Handle A and B inputs for state 0
		if(a){
			new_state = 1;
			speedLimit ++;
		}else if(b){
			new_state = 2;
			speedLimit --;
		}
	}
	else if (old_state == 1) {

	    // Handle A and B inputs for state 1
		if(b){
			new_state = 3;
			speedLimit ++;
		}else if(!a){
			new_state = 0;
			speedLimit --;
		}
	}
	else if (old_state == 2) {

	    // Handle A and B inputs for state 2
		if(a){
			new_state = 3;
			speedLimit --;
		}else if(!b){
			new_state = 0;
			speedLimit ++;
		}
	}
	else {   // old_state = 3

	    // Handle A and B inputs for state 3
		if(!a){
			new_state = 2;
			speedLimit ++;
		}else if(!b){
			new_state = 1;
			speedLimit --;
		}
	}
	if(speedLimit >= 99){
		speedLimit = 99;
	}else if(speedLimit <= 1){
		speedLimit = 1;
	}

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    changed = 1;
	    old_state = new_state;

	}
	
}

// end rotary encoder

// buzzer
void timer0_init(void){
    DDRC |= (1 << PC1); // set buzzer as output
    TCCR0A |= (0b01 << WGM01); // set bit to ctc
    TIMSK0 |= (1 << OCIE0A); // enable interrupt
    OCR0A = 70; // (16e6 / (256*2*440)) - 1;
}

ISR(TIMER0_COMPA_vect){
    buzzerCount ++;
    PORTC ^= (1 << PC1); // toggle
    if(buzzerCount >= buzzerMaxCount){
        PORTC &= ~(1 << PC1); // turn buzzer off
        buzzerCount = 0;
        isBuzzing = 0;
        TCCR0B &= ~(1 << CS02);
    }
}
// end buzzer

// servo 
void timer2_init(void){
    DDRB |= (1 << 3);
    TCCR2A |= (0b11 << WGM20);  // set to fast pwm
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    // OCR2A = 128;                // Initial pulse duty cycle of 50%
}

// serial
void tx_speed(int speed){
    lcd_moveto(0,0);
    char buffer[7];
    snprintf(buffer, 7, "{%d}", speed);
    int i;
    for(i=0; buffer[i] != '\0'; i++){
        while ((UCSR0A & (1 << UDRE0)) == 0)
        {
            // wait for tx register to be ready
        }
        UDR0 = buffer[i];
    }

}

ISR(USART_RX_vect){

    char received_char = UDR0;  // Read the received data

    if (received_char == '{') {
        data_started = 1;
        buffer_count = 0;
        data_valid = 0;
    } else if (data_started) {
        if (received_char >= '0' && received_char <= '9' && buffer_count < 4) {
            rxbuffer[buffer_count++] = received_char;
        } else if (received_char == '}' && buffer_count > 0) {
            rxbuffer[buffer_count] = '\0';  
            data_valid = 1;
            data_started = 0;
            display_speed();
        } else {
            // Invalid character or overflow, reset everything
            data_started = 0;
            buffer_count = 0;
            data_valid = 0;
        }
    }
}

void display_speed() {
    if (data_valid) {
        // Clear the flag first to prepare for next data
        data_valid = 0;

        // Set the position on the LCD where data will be displayed
        lcd_moveto(1, 10);

        //ead speed from buffer into a string and then convert to integer
        char data[6]; // ensure enough space for data and null terminator
        sscanf(rxbuffer, "%5s", data); // read at most 5 characters to prevent overflow
        lcd_stringout(data); // display the string on the LCD R

        // Convert string data to an integer
        uint16_t trans_speed = atoi(data);

        // Determine which LEDs to light up based on speed difference
        uint8_t mask = 0; // create a mask to change the LEDs accordingly
        if (abs(local_speed - trans_speed) <= 30) {
            mask = (1 << 2) | (1 << 3); // set both LEDs if speed difference is small
        } else if (local_speed > trans_speed) {
            mask = (1 << 2); // set only the first LED
        } else {
            mask = (1 << 3); // set only the second LED
        }

        // Apply mask to PORTC while preserving other bits
        PORTC &= ~((1 << 2) | (1 << 3)); // clear the bits for the two LEDs
        PORTC |= mask; // set the bits according to the mask
    }
}