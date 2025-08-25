//sending data from hcsro4 ultrasonic sensor to an arduino and then sending the same data to another arduino using uart

//1st arduino

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU / (16UL * BAUD)) - 1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define TRIG_PIN PB1
#define ECHO_PIN PB2
#define SOFT_TX_PIN PD3
#define SOFT_RX_PIN PD2

#define START_BYTE 0xFF
#define DISTANCE_CMD 0x01
#define END_BYTE 0xFE

volatile uint32_t timer1_overflow_count = 0;

void init_system(void);
void init_hardware_uart(void);
void init_timer1(void);
void uart_transmit(uint8_t data);
void soft_uart_transmit(uint8_t data);
void send_uart_packet(uint16_t distance);
uint32_t get_microseconds(void);
uint16_t read_hc_sr04(void);

int main(void) {
    init_system();
    const char* startup_msg = "Arduino #1 Ready - Sending to Arduino #2\r\n";
    for (int i = 0; startup_msg[i] != '\0'; i++) uart_transmit(startup_msg[i]);
    while (1) {
        uint16_t distance = read_hc_sr04();
        if (distance > 0 && distance < 400) {
            const char* msg = "Sending: ";
            for (int i = 0; msg[i] != '\0'; i++) uart_transmit(msg[i]);
            char dist_str[10];
            itoa(distance, dist_str, 10);
            for (int i = 0; dist_str[i] != '\0'; i++) uart_transmit(dist_str[i]);
            const char* unit = " cm\r\n";
            for (int i = 0; unit[i] != '\0'; i++) uart_transmit(unit[i]);
            send_uart_packet(distance);
        }
        _delay_ms(1000);
    }
}

void init_system(void) {
    cli();
    init_hardware_uart();
    init_timer1();
    DDRB |= (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    DDRD |= (1 << SOFT_TX_PIN);
    DDRD &= ~(1 << SOFT_RX_PIN);
    PORTB &= ~(1 << TRIG_PIN);
    PORTD |= (1 << SOFT_TX_PIN);
    sei();
}

void init_hardware_uart(void) {
    uint16_t baud_setting = (F_CPU / (16UL * 9600)) - 1;
    UBRR0H = (uint8_t)(baud_setting >> 8);
    UBRR0L = (uint8_t)baud_setting;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void init_timer1(void) {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TIMSK1 |= (1 << TOIE1);
    timer1_overflow_count = 0;
}

ISR(TIMER1_OVF_vect) {
    timer1_overflow_count++;
}

void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void soft_uart_transmit(uint8_t data) {
    OCR1A = TCNT1 + 208;
    PORTD &= ~(1 << SOFT_TX_PIN);
    while (!(TIFR1 & (1 << OCF1A)));
    TIFR1 |= (1 << OCF1A);
    for (uint8_t i = 0; i < 8; i++) {
        OCR1A += 208;
        if (data & (1 << i)) PORTD |= (1 << SOFT_TX_PIN);
        else PORTD &= ~(1 << SOFT_TX_PIN);
        while (!(TIFR1 & (1 << OCF1A)));
        TIFR1 |= (1 << OCF1A);
    }
    OCR1A += 208;
    PORTD |= (1 << SOFT_TX_PIN);
    while (!(TIFR1 & (1 << OCF1A)));
    TIFR1 |= (1 << OCF1A);
}

uint32_t get_microseconds(void) {
    uint32_t m;
    uint16_t t;
    cli();
    m = timer1_overflow_count;
    t = TCNT1;
    sei();
    return ((m * 65536) + t) / 2;
}

uint16_t read_hc_sr04(void) {
    uint32_t start_time, pulse_duration;
    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);
    uint32_t timeout = get_microseconds() + 30000;
    while (!(PINB & (1 << ECHO_PIN))) if (get_microseconds() > timeout) return 0;
    start_time = get_microseconds();
    timeout = start_time + 30000;
    while (PINB & (1 << ECHO_PIN)) if (get_microseconds() > timeout) return 0;
    pulse_duration = get_microseconds() - start_time;
    return (pulse_duration * 343) / (2 * 10000);
}

void send_uart_packet(uint16_t distance) {
    uint8_t high_byte = (distance >> 8) & 0xFF;
    uint8_t low_byte = distance & 0xFF;
    soft_uart_transmit(START_BYTE);
    soft_uart_transmit(DISTANCE_CMD);
    soft_uart_transmit(high_byte);
    soft_uart_transmit(low_byte);
    soft_uart_transmit(END_BYTE);
}


//2nd arduino

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define LED_PIN PB5
#define SOFT_RX_PIN PD2
#define START_BYTE 0xFF
#define DISTANCE_CMD 0x01
#define END_BYTE 0xFE

#define RXBUF_SIZE 64
volatile uint8_t rxbuf[RXBUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

volatile uint8_t rx_active = 0;
volatile uint8_t rx_bit = 0;
volatile uint8_t rx_byte = 0;

void uart_init(void){
    uint16_t b = (F_CPU/(16UL*9600))-1;
    UBRR0H = (uint8_t)(b>>8);
    UBRR0L = (uint8_t)b;
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}
void uart_tx(uint8_t c){
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}
void uart_print(const char* s){
    while(*s) uart_tx(*s++);
    uart_tx('\r'); uart_tx('\n');
}

static inline void rb_push(uint8_t b){
    uint8_t n = (rx_head+1) & (RXBUF_SIZE-1);
    if(n!=rx_tail){ rxbuf[rx_head]=b; rx_head=n; }
}

void softuart_rx_init(void){
    DDRD &= ~(1<<SOFT_RX_PIN);
    PORTD |= (1<<SOFT_RX_PIN);
    TCCR1A = 0;
    TCCR1B = (1<<CS11);
    EICRA = (1<<ISC01);
    EIMSK = (1<<INT0);
    TIMSK1 &= ~(1<<OCIE1A);
}

ISR(INT0_vect){
    if(rx_active) return;
    rx_active = 1;
    rx_bit = 0;
    rx_byte = 0;
    OCR1A = TCNT1 + 312;
    TIFR1 |= (1<<OCF1A);
    TIMSK1 |= (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect){
    if(rx_bit < 8){
        if(PIND & (1<<SOFT_RX_PIN)) rx_byte |= (1<<rx_bit);
        rx_bit++;
        OCR1A += 208;
    }else{
        (void)PIND;
        TIMSK1 &= ~(1<<OCIE1A);
        rx_active = 0;
        rb_push(rx_byte);
    }
}

int main(void){
    uart_init();
    DDRB |= (1<<LED_PIN);
    PORTB &= ~(1<<LED_PIN);
    softuart_rx_init();
    sei();

    uint8_t state = 0;
    uint8_t cmd = 0;
    uint8_t hb = 0, lb = 0;

    uart_print("Arduino #2 Ready");

    for(;;){
        if(rx_head != rx_tail){
            uint8_t b = rxbuf[rx_tail];
            rx_tail = (rx_tail+1) & (RXBUF_SIZE-1);

            switch(state){
                case 0: if(b==START_BYTE){ state=1; } break;
                case 1: cmd=b; state=2; break;
                case 2: hb=b; state=3; break;
                case 3: lb=b; state=4; break;
                case 4:
                    if(b==END_BYTE && cmd==DISTANCE_CMD){
                        uint16_t dist = ((uint16_t)hb<<8)|lb;
                        char s[16];
                        itoa(dist, s, 10);
                        uart_tx('D'); uart_tx(':'); uart_tx(' ');
                        for(char* p=s; *p; ++p) uart_tx(*p);
                        uart_tx(' '); uart_tx('c'); uart_tx('m'); uart_tx('\r'); uart_tx('\n');
                        PORTB ^= (1<<LED_PIN);
                    }
                    state=0;
                    break;
                default: state=0; break;
            }
        }
    }
}

