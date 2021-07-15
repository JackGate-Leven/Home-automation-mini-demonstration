#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// -- Source CAB202 Topic 8 - Serial Communication
#define SET_BIT(reg, pin) (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin) (reg) &= ~(1 << (pin))
#define BIT_VALUE(reg, pin) (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin) (BIT_VALUE((reg),(pin))==1) 

// timer definitions
#define FREQ (16000000)
#define PRESCALE (1024)

//uart definitions
// -- Source CAB202 Topic 8 - Serial Communications
#define MYUBRR (16000000/16/9600-1)

// declare main functions
void setup(void);
int main(void);
int getDistance(void);
void potentiometer_change(void);
void pwm_write(uint8_t duration);
void if_button_press(void);
void pwm_init(uint16_t division_factor);
void uart_put_string(char transmit_data[]);
void uart_putbyte(unsigned char data);
int uart_getbyte(unsigned char *buffer);

char*  itoa( int value, char* str, int base );


// LCD setup and definitions
///////////////////////////////
// -- Source CAB202 Topic 11 - LCD

#define LCD_USING_4PIN_MODE (1)

#define LCD_DATA4_DDR (DDRD)
#define LCD_DATA5_DDR (DDRD)
#define LCD_DATA6_DDR (DDRD)
#define LCD_DATA7_DDR (DDRD)

#define LCD_DATA4_PORT (PORTD)
#define LCD_DATA5_PORT (PORTD)
#define LCD_DATA6_PORT (PORTD)
#define LCD_DATA7_PORT (PORTD)

#define LCD_DATA4_PIN (4)
#define LCD_DATA5_PIN (5)
#define LCD_DATA6_PIN (6)
#define LCD_DATA7_PIN (7)

#define LCD_RS_DDR (DDRB)
#define LCD_ENABLE_DDR (DDRB)
#define LCD_RS_PORT (PORTB)
#define LCD_ENABLE_PORT (PORTB)
#define LCD_RS_PIN (5)
#define LCD_ENABLE_PIN (4)

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
 #define LCD_BLINKOFF 0x00
// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);
void lcd_clear(void);
void lcd_home(void);
void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t);
void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);
size_t lcd_write(uint8_t);
void lcd_command(uint8_t);
void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);
uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;

void lcd_init(void){
	//dotsize
	if (LCD_USING_4PIN_MODE){
		_lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	} else {
		_lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
	}

	_lcd_displayfunction |= LCD_2LINE;
  
	// RS Pin
	LCD_RS_DDR |= (1 << LCD_RS_PIN);
	// Enable Pin
	LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);

	#if LCD_USING_4PIN_MODE
		//Set DDR for all the data pins
		LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
		LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
		LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
		LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
	#else
		//Set DDR for all the data pins
		LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
		LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
		LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
		LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
		LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
		LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
		LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
		LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
	#endif
	// delay until lcd can get proper voltage on startup
	_delay_us(50000);
	// Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);

	//put the LCD into 4 bit or 8 bit mode
	if (LCD_USING_4PIN_MODE) {
		// this is according to the hitachi HD44780 datasheet
		// figure 24, pg 46
		// we start in 8bit mode, try to set 4 bit mode
		lcd_write4bits(0b0111);
		_delay_us(4500); // wait min 4.1ms
		// second try
		lcd_write4bits(0b0111);
		_delay_us(4500); // wait min 4.1ms

		// third go!
		lcd_write4bits(0b0111);
		_delay_us(150);

		// finally, set to 4-bit interface
		lcd_write4bits(0b0010);
	} else {
	 // this is according to the hitachi HD44780 datasheet
	 // page 45 figure 23
	 // Send function set command sequence
	 lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
	 _delay_us(4500); // wait more than 4.1ms
	 // second try
	 lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
	 _delay_us(150);
	 // third go
	 lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
	 }
	 // finally, set # lines, font size, etc.
	 lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
	 // turn the display on with no cursor or blinking default
	 _lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	 lcd_display();
	 // clear it off
	 lcd_clear();
	 // Initialize to default text direction (for romance languages)
	 _lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	 // set the entry mode
	 lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[]){
	lcd_setCursor(x,y);
	for(int i=0; string[i]!='\0'; ++i){
		lcd_write(string[i]);
	}
}
void lcd_write_char(uint8_t x, uint8_t y, char val){
	lcd_setCursor(x,y);
	lcd_write(val);
}

void lcd_clear(void){
	lcd_command(LCD_CLEARDISPLAY); // clear display, set cursor position to zero
	_delay_us(2000); // this command takes a long time!
}

void lcd_home(void){
	lcd_command(LCD_RETURNHOME); // set cursor position to zero
	_delay_us(2000); // this command takes a long time!
}

void lcd_setCursor(uint8_t col, uint8_t row){
	if ( row >= 2 ) {
		row = 1;
	}

	lcd_command(LCD_SETDDRAMADDR | (col + row*0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
	_lcd_displaycontrol &= ~LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_display(void) {
	_lcd_displaycontrol |= LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
	_lcd_displaycontrol &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_cursor(void) {
	_lcd_displaycontrol |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
	_lcd_displaycontrol &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_blink(void) {
	_lcd_displaycontrol |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
	//
	lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
	lcd_send(value, 1);
	return 1; // assume sucess
}
/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) {
	//RS Pin
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	LCD_RS_PORT |= (!!mode << LCD_RS_PIN);
	if (LCD_USING_4PIN_MODE) {
	lcd_write4bits(value>>4);
	lcd_write4bits(value);
	} else {
	lcd_write8bits(value);
	}
}

void lcd_pulseEnable(void) {
	//Enable Pin
	LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
	_delay_us(1);
	LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
	_delay_us(1); // enable pulse must be >450ns
	LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
	_delay_us(100); // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) {
	//Set each wire one at a time
	LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
	LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
	value >>= 1;
	LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
	LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
	value >>= 1;
	LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
	LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
	value >>= 1;
	LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
	LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);
	lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) {
	//Set each wire one at a time
	#if !LCD_USING_4PIN_MODE
	LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
	LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
	value >>= 1;
	LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
	LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
	value >>= 1;
	LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
	LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
	value >>= 1;
	LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
	LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
	value >>= 1;
	LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
	LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
	value >>= 1;
	LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
	LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
	value >>= 1;
	LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
	LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
	value >>= 1;
	LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
	LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

	lcd_pulseEnable();
	#endif
}

//////////////////////////////// End of LCD setup

// global variables
int fan_speed = 100;
int lightbulb_intensity = 100;
int timer = 0;
int timer_status = 0;
double timer_start;
double timer_end;
double timer_value;
double current_time;

uint16_t pot_value;
char temporary[16];
unsigned char input_byte;
int mapped_duty_cycle;
int display_pot;

volatile uint8_t switch_counter = 0;
volatile uint8_t pressed = 0;
uint8_t switch_counter_mask = 0b00111111;

// 0 is light bulb, 1 is fan
int current_device = 0;

// on/off startup, lightbulb off
int switch_on_off = 1;
int led_on_off = 0;
int fan_on_off = 0;
int lightbulb_on_off = 0;



// this function is called at the startup of the system, it sets up I/O pins
// registers and runs through intial sequence
// parameter void, return void
void setup(void){

	// enable pin C0 for input from potentiometer
	CLEAR_BIT(DDRC, 0);

	// enable pins C1 for input from the device button
	CLEAR_BIT(DDRC, 1);

	// enable pin C3 for output to the On/Off LED
	SET_BIT(DDRC, 3);

	// enable pin C5 for input from the ON/OFF switch
	CLEAR_BIT(DDRC, 5);

	// enable pin B2 for output to the light bulb
	SET_BIT(DDRB, 1);

	// enable pin B1 for output to the fan motor
	SET_BIT(DDRB, 2);	

	// setup uart
	UBRR0H = (unsigned char)(MYUBRR>>8);
    UBRR0L = (unsigned char)(MYUBRR);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);

	// initialise adc
	// ADC Enable and pre-scaler of 128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX = (1 << REFS0);

	// Timer 2 in normal mode, with pre-scaler 1024 ==> ~61Hz overflow.
	TCCR2A = 0;
    TCCR2B = 5;
    TIMSK2 = 1;

    // Timer 1 
    // OCR1A = 128; // set PWM for 50% duty cycle
    // TCCR1A |= (1 << COM1A1); // set none-inverting mode
    // // TinkerCAD Errata: timer clocking must be enabled before WGM // set prescaler to 8 and starts PWM TCCR0B = (1 << CS01);
    // TCCR1A |= (1 << WGM01) | (1 << WGM00); // set fast PWM Mode

	// // Enable timer overflow, and turn on interrupts.
	sei();

	// set up lcd and display start up sequence
	lcd_init();
	lcd_write_string(0, 0, "Welcome to");
	lcd_write_string(0, 1, "Home.io");

	uart_put_string("#############################################");
    uart_putbyte('\n');
    uart_put_string("welcome to Home.IO's timer control monitor.");
    uart_putbyte('\n');
    uart_put_string("You can specify between 1-9 hours before all connected devices are turned off.");
    uart_putbyte('\n');
    uart_put_string("Please enter a whole number from 1-9.");
    uart_putbyte('\n');
    uart_put_string("#############################################");
    uart_putbyte('\n');
	
	lcd_clear();
	lcd_write_string(0, 0, "Lightbulb");
	lcd_write_string(0, 1, "100");
}

volatile int overflow_counter = 0;

//interrupt service timer setup
// -- SOURCE - CAB202 Topic 9 - Debouncing, Timers, and Interrupts
ISR(TIMER2_OVF_vect) {
	switch_counter = ((switch_counter << 1) & switch_counter_mask) | BIT_VALUE(PINC, 1);

    if (switch_counter == 0) pressed = 0;
    else if (switch_counter == switch_counter_mask) pressed = 1;

    overflow_counter ++;
}

// -- SOURCE - CAB202 Topic 8 - Serial communication
void uart_put_string(char transmit_data[]) {
  int i = 0;

  while (transmit_data[i] != 0){
    uart_putbyte(transmit_data[i]);
    i++;
  }

  uart_putbyte(0);
}

// this function transmit a single byte through UART
// parameter unsigned char *buffer, return int byte recieved
// -- SOURCE - CAB202 Topic 8 - Serial communication
void uart_putbyte(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // Send data by assigning into UDR0
    UDR0 = data;
}

// this function recieves a single byte through UART
// parameter unsigned char *buffer, return int byte recieved
// -- SOURCE - CAB202 Topic 8 - Serial communication
int uart_getbyte(unsigned char *buffer) {
    // If receive buffer contains data...
    if (UCSR0A & (1 << RXC0)) {
        // Copy received byte from UDR0 into memory location (*buffer)
        *buffer = UDR0;
        // 
        return 1;
    }
    else {
        return 0;
    }
}

// this function changes the duty cycle of the pwm timer
// uint8_t duration paramater, void return
// -- SOURCE - CAB202 Topic 10 – Analog to Digital Conversion and Pulse-Width Modulation
void pwm_write(uint8_t duration) {
    OCR0A = duration;
}

// this function is called to signal the device change button was pressed
// void paramater, void return
void if_button_press(void){
	lcd_clear();
	
	// if the device is the fan, change to the lightbulb
	if (current_device == 1){
		current_device = 0;
		potentiometer_change();
		if (lightbulb_on_off == 1){
			led_on_off = 1;
			SET_BIT(PORTC, 3);
		} else {
			led_on_off = 0;
			CLEAR_BIT(PORTC, 3);
		}
	} else { // if the device is the lightbulb, change to the fan
		current_device = 1;
		potentiometer_change();
		if (fan_on_off == 1){
			led_on_off = 1;
			SET_BIT(PORTC, 3);
		} else {
			led_on_off = 0;
			CLEAR_BIT(PORTC, 3);
		}
	}
}



// this function is called to signal a change in the potentiometer's return value or to reset the displayed value
// void paramater, void return
void potentiometer_change(void){
	lcd_clear();
	display_pot = pot_value / 2.5;
	if (display_pot < 0){
		display_pot = 0;
	}

	if (current_device == 0){
  		lightbulb_intensity = pot_value;
		lcd_write_string(0, 0, "Lightbulb");
		itoa(display_pot,temporary,10);
		lcd_write_string(0, 1, temporary);
		if (lightbulb_on_off == 1){
			//analogWrite(9, lightbulb_intensity);
			// I could not implement PWM
			SET_BIT(PORTB, 1);
		} else {
			CLEAR_BIT(PORTB, 1);
		}
  	}

  	else if (current_device == 1){
  		fan_speed = pot_value;
		lcd_write_string(0, 0, "Fan");
		itoa(display_pot,temporary,10);
		lcd_write_string(0, 1, temporary);
		if (fan_on_off == 1){
			//analogWrite(10, fan_speed);
			// I could not implement PWM
			SET_BIT(PORTB, 2);
		} else {
			CLEAR_BIT(PORTB, 2);
		}
  	}
}

void on_off_switch_change(void){
	// lightbulb
	if (current_device == 0){
		// off to on
		if (lightbulb_on_off == 0){
			lightbulb_on_off = 1;
			SET_BIT(PORTB, 1);
			//analogWrite(9, lightbulb_intensity);
			SET_BIT(PORTC, 3);
		} else {
			lightbulb_on_off = 0;
			CLEAR_BIT(PORTB, 1);
			CLEAR_BIT(PORTC, 3);
		}
	}
	// fan
	if (current_device == 1){
		// off to on
		if (fan_on_off == 0){
			fan_on_off = 1;
			SET_BIT(PORTB, 2);
			//analogWrite(10, fan_speed);
			SET_BIT(PORTC, 3);
		} else {
			fan_on_off = 0;
			CLEAR_BIT(PORTB, 2);
			CLEAR_BIT(PORTC, 3);
		}
	}
}

void process(void) {
	// check whether on/off led should be on or off
	led_on_off = (current_device == 0) ? ((lightbulb_on_off == 1) ? 0 : 1) : ((fan_on_off == 1) ? 0 : 1);
	uint8_t prevState = 0;
	// get value of potentiometer using ADC
	// Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);
	// Wait for ADSC bit to clear, signalling conversion complete.
	while ( ADCSRA & (1 << ADSC) ) {}
	// Result now available in ADC
	pot_value = ADC/4;


	// if timer hits 0 then turn off all devices
	if (timer_status == 1){
		//compute elapsed time
		double current_time = ( overflow_counter * 256.0 + TCNT2 ) * PRESCALE  / FREQ / 8;
		// current_time /= 10;

		// if the timer has been has been reached or gone beyond
		if ((current_time - timer_start) > timer_value){
			uart_put_string("timer has finished");
			uart_putbyte('\n');
			timer_status = 0;
			lightbulb_on_off = 0;
			fan_on_off = 0;
			led_on_off = 0;
			CLEAR_BIT(PORTB, 1);
			CLEAR_BIT(PORTB, 2);
			CLEAR_BIT(PORTC, 3);
			potentiometer_change();
		}
	}


	if (current_device == 0){
  		if (lightbulb_on_off == 1){
  			SET_BIT(PORTC, 3);
  		} else {
  			CLEAR_BIT(PORTC, 3);
  		}
  	} else {
  		if (fan_on_off == 1){
  			SET_BIT(PORTC, 3);
  		} else {
  			CLEAR_BIT(PORTC, 3);
  		}
  	}

  	// trigger change in potentiometer from movement of the potentiometer
  	if (((current_device == 0 && (pot_value != lightbulb_intensity)) 
  		|| (current_device == 1 && (pot_value != fan_speed)))){
  		potentiometer_change();
  	}

	// toggle device on/off from switch state change
	if (BIT_VALUE(PINC, 5) != switch_on_off){
		switch_on_off = (switch_on_off == 0) ? 1 : 0;
		on_off_switch_change();
	}

	// if change device button is pressed - debounce
	// attempt at interrupt based debounce
	if (pressed != prevState) {
        prevState = pressed;
        if (pressed == 1){
        	if_button_press();
        }      
        // delay based debounce
    	_delay_ms(200);
    } 
    
    // for the sake of demonstration, the timer has been scaled down by 2 factors so that 
    // the timer is considering input as seconds instead of hours
    // if the user inputs something through UART
    if (uart_getbyte(&input_byte) && timer_status == 0){ 
    	int output = (int) input_byte;

    	// test if user input is a number 1-9
    	if (output > 48 && output < 58){
    		timer_status = 1;
    		timer_start =  ( overflow_counter * 256.0 + TCNT2 ) * PRESCALE  / FREQ / 8;
    		timer_value = output-48;
    		uart_put_string("A timer has been set for");
    		uart_putbyte(timer_value + 48);
    		uart_put_string(" hour/s.");
    		uart_putbyte('\n');
    	} else {
    		uart_put_string("Setting the timer failed, try entering a number from 0-9");
    		uart_putbyte('\n');
    	}
    }
}

// !!!!!!!!!!!!!!!! needs to be 'void loop (void)' instead of main in order to run properly on tinkercad
int main(void) {
	setup();

	for ( ;; ) {
		process();
		_delay_ms(100);
	}
}