#include"avr/io.h"
#include"avr/pgmspace.h"
#include"avr/wdt.h"
#include"avr/eeprom.h"
#include"avr/interrupt.h"

#define baudrate 9600
#define baudprescaler ((F_CPU)/(baudrate*16UL)-1)

#define sensor_encoding_factor 4
#define enable_calibration

#define led0 PIND5
#define led1 PIND6
#define led2 PIND7
#define output0 PIND4
#define cs_sensor0 PINB2
#define cs_sensor1 PINB1
#define spi_clock PINB5
#define spi_mosi PINB3

#define no_faultcode 0
#define brownout_faultcode 1
#define wdt_faultcode 2
#define badint_faultcode 3
#define sensor0_faultcode 4
#define sensor1_faultcode 5

typedef enum { false, true } bool;

void software_reset(uint8_t code);
void readout();
void calibrate_sensors();
void uart_to_eeprom();
void uart_send_message(uint8_t start, uint8_t size);
uint16_t read_spi(uint8_t slave, bool chaining);
float read_temperature(uint8_t sensor, bool normalconversion);

float EEMEM sensor1_calib_scaling = 1;
float EEMEM sensor1_calib_offset = 0;
float EEMEM E_delta_t_switch_off = 5;
float EEMEM E_delta_t_switch_on = 5;
uint8_t EEMEM faultcode = 0;//keep order
char EEMEM messages[] = {'o','k','w','a','i','t','i','n','g'};//needed? more?

struct{
	float delta_t_switch_off;
	float delta_t_switch_on;
}temperature_settings;

uint8_t wdt_source_known = false;

//turns on output if sensor0 is significantly higher than sensor1

//stack size big enough?
//line break characters?
//eeprom wear?
//wat was that with de wdt reset before turing it of again?
//check software reset?
//ints are turned off in ints wdt int cant fire
//catch for reset loop disconnected sensor

int main(){
	//setup watchdog, reset, 8s
	WDTCSR = 1<<WDCE | 1<<WDE;
	WDTCSR = 1<<WDP3 | 1<<WDIE | 1<<WDP0 | 1<<WDE;

	//reduce power
	PRR = 0b11100001; //disable twi, adc, tim2

	PORTB = 1<<PINB0 | 1<<PINB6 | 1<<PINB7;
	PORTC = 1<<PINC4 | 1<<PINC5;
	PORTD = 1<<PIND0 | 1<<PIND1 | 1<<PIND2 |1<<PIND3;
	DDRB = 1<<cs_sensor0 | 1<<cs_sensor1 | 1<<spi_clock | 1<<spi_mosi;
	DDRD = 1<<PIND0 | 1<<PIND1 | 1<<PIND2 | 1<<output0 | 1<<led0 | 1<<led1| 1<<led2;//1 for output

	//setup spi
	SPCR = 1<<SPE | 1<<MSTR | 1<<SPR0 | 1<<SPR1;

	//setup usart
	UCSR0B = 0b10011000;//rxi,txen, rxen
	UCSR0C = 0b00001110;//8 bit , 2 stop bit, asynchronous, disabled partity
	UBRR0L = baudprescaler;

	//setup timer
	TCCR1B = 0b00001101;
	TIMSK1 = 0b00000010;
	OCR1A = 65000;

	if(MCUSR & 1<<BORF){
		eeprom_write_byte((uint8_t*)&faultcode, brownout_faultcode);
	}

	MCUSR = 0;

	if(eeprom_read_byte((const uint8_t*)&faultcode) != 0){
		PORTD |= 1<<led0;
	}
	//while(eeprom_read_byte((const uint8_t*)&faultcode) != 0), nonvol fault code needed then

	sei();

	eeprom_read_block((void*)&temperature_settings, (const void*)&E_delta_t_switch_off, sizeof(temperature_settings));//copy tempsets to ram

	while(true){
		wdt_reset();//startup executed fast enough?
	}
	return 0;
}

void readout(){
	//if the sensor discrds conversions during readout this needs a delay

	//turn off timint
	TIMSK1 = 0;
	struct {
		float sensor1_calib_scaling;
		float sensor1_calib_offset;
		float E_delta_t_switch_off;
		float E_delta_t_switch_on;
		uint8_t faultcode;
		uint16_t sensor0_couple;
		uint16_t sensor0_local;
		uint16_t sensor1_couple;
		uint16_t sensor1_local;
		uint8_t pinb;
		uint8_t pinc;
		uint8_t pind;
	} temp;

	eeprom_read_block((void*)&temp, 0, 17);

	temp.sensor0_couple = read_spi(cs_sensor0, true);
	temp.sensor0_local = read_spi(cs_sensor0, false);
	temp.sensor1_couple = read_spi(cs_sensor1, true);
	temp.sensor1_local = read_spi(cs_sensor1, false);

	temp.pinb = PINB;
	temp.pinc = PINC;
	temp.pind = PIND;

	for(uint8_t i=0; i<sizeof(temp); i++){
		UDR0 = *(uint8_t*)(&temp+i);
		while (!( UCSR0A & (1<<UDRE0))){}
	}

	//turn on timint
	TIMSK1 = 0b00000010;//if in the meantime an intterupt flag was set, it wil execute after this
}

void software_reset(uint8_t code){
	eeprom_write_byte((uint8_t*)&faultcode, code);
	wdt_source_known = true;
	while(true){
		//wait for wdt reset
	}
}

void uart_send_message(uint8_t start, uint8_t size){
	uint8_t temp[size];
	eeprom_read_block((void*)&temp, (const void*)start, size);//is this warning serious?

	for(uint8_t i=0; i<size; i++){
		UDR0 = temp[i];
		while (!( UCSR0A & (1<<UDRE0))){}
	}//share this code?
}

void uart_to_eeprom(){//write 17 bytes to eeprom
	//turn off interrupts and wdt
	cli();
	wdt_reset();
	WDTCSR = 1<<WDCE | 1<<WDE;
	WDTCSR = 0;

	uint8_t temp[17];
	uint8_t i = 0;

	uart_send_message(2,7);

	while(i < sizeof(temp)){
		while(!(UCSR0A & (1<<RXC0))){}
		eeprom_write_byte((uint8_t*)i, UDR0);
		UCSR0A &= ~(1<<RXC0);
		i++;
	}

	//readout();

	WDTCSR = 1<<WDCE | 1<<WDE;
	WDTCSR = 1<<WDP3 | 1<<WDIE | 1<<WDP0 | 1<<WDE;
	sei();//reen wdt and ints
}

void calibrate_sensors(){
	//sensor0 is assumed to be correct
	//turn off interrupts and wdt
	cli();
	wdt_reset();
	WDTCSR = 1<<WDCE | 1<<WDE;
	WDTCSR = 0;

	float sensor_0_low = read_temperature(cs_sensor0, false);
	float sensor_1_low = read_temperature(cs_sensor1, false);

	uart_send_message(2,7);

	while(!(UCSR0A & (1<<RXC0))){}
	UCSR0A &= ~(1<<RXC0);

	float sensor_0_high = read_temperature(cs_sensor0, false);
	float sensor_1_high = read_temperature(cs_sensor1, false);

	struct {
		float scaler;
		float offset;
	} temp;

	temp.scaler = (sensor_0_high-sensor_0_low)/(sensor_1_high-sensor_1_low);
	temp.offset = -sensor_1_low*temp.scaler+sensor_0_low;

	eeprom_write_block((void*)&temp.offset, (void*)&sensor1_calib_offset, 4);
	eeprom_write_block((void*)&temp.scaler, (void*)&sensor1_calib_scaling, 4);

	for(uint8_t i=0; i<sizeof(temp); i++){
		UDR0 = *(uint8_t*)(&temp+i);
		while (!( UCSR0A & (1<<UDRE0))){}
	}

	//readout();

	WDTCSR = 1<<WDCE | 1<<WDE;
	WDTCSR = 1<<WDP3 | 1<<WDIE | 1<<WDP0 | 1<<WDE;
	sei();//reen wdt and ints
}

ISR(USART_RX_vect){
	switch UDR0{
	case 'e':
		eeprom_write_byte((uint8_t*)&faultcode, 0);
		break;
	case 'r':
		readout();
		break;
	case 'c':
		calibrate_sensors();
		break;
	case 'm':
		uart_to_eeprom();
		break;
	}
}

ISR(__vector_default){//jump directly to this instead of via bad_interrupt?
	software_reset(badint_faultcode);
}

ISR(WDT_vect){
	if(wdt_source_known == false){
		eeprom_write_byte((uint8_t*)&faultcode, wdt_faultcode);
	}
}

ISR(TIMER1_COMPA_vect){
	if(PINC & (1<<output0)){
		if(read_temperature(cs_sensor0, true)-temperature_settings.delta_t_switch_off < read_temperature(cs_sensor1, true)){
			PINC = 1<<output0;
		}
	}else{
		if(read_temperature(cs_sensor0, true)-temperature_settings.delta_t_switch_on > read_temperature(cs_sensor1, true)){
			PINC = 1<<output0;
		}
	}
}

uint16_t read_spi(uint8_t slave, bool chaining){
	//optimize?
	uint8_t temp;

	PORTB &= (1<<slave)^0b11111111;
	SPDR=0xFF;
	while(!(SPSR & (1<<SPIF)));
	temp=SPDR;
	SPDR=0xFF;
	while(!(SPSR & (1<<SPIF)));

	if(!chaining){
		PORTB|=1<<slave;//this triggers moving converion to output(?), value to be read out next time
	}
	return(temp<<8)|SPDR;
}

float read_temperature(uint8_t sensor, bool normalconversion){//data ordering?
	uint16_t temp = read_spi(sensor, false);
	if(normalconversion && (temp & 1)){
		uint8_t temp = sensor0_faultcode;

		if(sensor == cs_sensor1){
			temp = sensor1_faultcode;
		}

		software_reset(temp);
	}
	//do away with non-data bits
	temp >>= 2;
	//convert to twos complement
	if(temp & 0b0010000000000000){
		temp &= 0b1101111111111111;
		temp = (temp^0b1111111111111111) + 1;
	}

	float output = ((float)temp)/sensor_encoding_factor;

#ifdef enable_calibration
	if(normalconversion && (sensor == cs_sensor1)){
		float offset;
		float scaling;
		eeprom_read_block((void*)&scaling, (const void*)&sensor1_calib_scaling, 4);
		eeprom_read_block((void*)&offset, (const void*)&sensor1_calib_offset, 4);
		output = output * scaling + offset;
	}
#endif

	return output;
}
