//sending data from hcsro4 ultrasonic sensor to an arduino and then sending the same data to another arduino using uart

//1st arduino

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pin definitions (Arduino Uno mapping)
#define TRIG_PIN PB1        // Pin 9 (PORTB bit 1)
#define ECHO_PIN PB2        // Pin 10 (PORTB bit 2)
#define SOFT_TX_PIN PD3     // Pin 3 for SoftwareSerial TX
#define SOFT_RX_PIN PD2     // Pin 2 for SoftwareSerial RX (not used in sender)

// UART Protocol constants
#define START_BYTE 0xFF
#define DISTANCE_CMD 0x01
#define END_BYTE 0xFE

// Timer/Counter variables for microsecond timing
volatile uint32_t timer1_overflow_count = 0;

// Function prototypes
void init_system(void);
void init_hardware_uart(void);
void init_software_uart(void);
void init_timer1(void);
void uart_transmit(uint8_t data);
void soft_uart_transmit(uint8_t data);
uint32_t get_microseconds(void);
uint16_t read_hc_sr04(void);
void send_uart_packet(uint16_t distance);

int main(void) {
    init_system();
    
    // Send startup message
    const char* startup_msg = "Arduino #1 Ready - Sending to Arduino #2\r\n";
    for (int i = 0; startup_msg[i] != '\0'; i++) {
        uart_transmit(startup_msg[i]);
    }
    
    while (1) {
        uint16_t distance = read_hc_sr04();
        
        if (distance > 0 && distance < 400) {
            // Send distance via hardware UART (Serial Monitor)
            const char* msg = "Sending: ";
            for (int i = 0; msg[i] != '\0'; i++) {
                uart_transmit(msg[i]);
            }
            
            // Convert distance to string and send
            char dist_str[10];
            itoa(distance, dist_str, 10);
            for (int i = 0; dist_str[i] != '\0'; i++) {
                uart_transmit(dist_str[i]);
            }
            
            const char* unit = " cm\r\n";
            for (int i = 0; unit[i] != '\0'; i++) {
                uart_transmit(unit[i]);
            }
            
            // Send via software UART to Arduino #2
            send_uart_packet(distance);
        }
        
        _delay_ms(1000);
    }
    
    return 0;
}

// Initialize system registers
void init_system(void) {
    // Disable interrupts during initialization
    cli();
    
    // Initialize hardware
    init_hardware_uart();
    init_software_uart();
    init_timer1();
    
    // Set pin directions
    DDRB |= (1 << TRIG_PIN);    // Trigger pin as output
    DDRB &= ~(1 << ECHO_PIN);   // Echo pin as input
    DDRD |= (1 << SOFT_TX_PIN); // Software UART TX as output
    DDRD &= ~(1 << SOFT_RX_PIN);// Software UART RX as input (not used)
    
    // Set initial pin states
    PORTB &= ~(1 << TRIG_PIN);  // Trigger pin LOW
    PORTD |= (1 << SOFT_TX_PIN);// Software UART TX idle HIGH
    
    // Enable global interrupts
    sei();
}

// Initialize hardware UART (for Serial Monitor)
void init_hardware_uart(void) {
    uint16_t baud_setting = (F_CPU / (16UL * 9600)) - 1;
    
    // Set baud rate
    UBRR0H = (uint8_t)(baud_setting >> 8);
    UBRR0L = (uint8_t)baud_setting;
    
    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Initialize software UART timing
void init_software_uart(void) {
    // Software UART will use bit-banging with precise timing
    // 9600 baud = ~104 microseconds per bit
}

// Initialize Timer1 for microsecond timing
void init_timer1(void) {
    // Set Timer1 to normal mode, prescaler 8 (2MHz at 16MHz)
    TCCR1A = 0;
    TCCR1B = (1 << CS11);  // Prescaler 8
    
    // Enable Timer1 overflow interrupt
    TIMSK1 |= (1 << TOIE1);
    
    timer1_overflow_count = 0;
}

// Timer1 overflow interrupt
ISR(TIMER1_OVF_vect) {
    timer1_overflow_count++;
}

// Hardware UART transmit
void uart_transmit(uint8_t data) {
    // Wait for transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0)));
    
    // Send data
    UDR0 = data;
}

// Software UART transmit (9600 baud bit-banging)
void soft_uart_transmit(uint8_t data) {
    uint8_t i;
    
    // Start bit (LOW)
    PORTD &= ~(1 << SOFT_TX_PIN);
    _delay_us(104);  // 1/9600 ≈ 104 microseconds
    
    // Data bits (LSB first)
    for (i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            PORTD |= (1 << SOFT_TX_PIN);   // HIGH for '1'
        } else {
            PORTD &= ~(1 << SOFT_TX_PIN);  // LOW for '0'
        }
        _delay_us(104);
    }
    
    // Stop bit (HIGH)
    PORTD |= (1 << SOFT_TX_PIN);
    _delay_us(104);
}

// Get current time in microseconds
uint32_t get_microseconds(void) {
    uint32_t m;
    uint16_t t;
    
    cli();
    m = timer1_overflow_count;
    t = TCNT1;
    sei();
    
    return ((m * 65536) + t) / 2;  // Convert to microseconds (2MHz timer)
}

// Read HC-SR04 sensor using bare metal
uint16_t read_hc_sr04(void) {
    uint32_t start_time, pulse_duration;
    uint16_t distance;
    
    // Send trigger pulse
    PORTB &= ~(1 << TRIG_PIN);  // LOW
    _delay_us(2);
    
    PORTB |= (1 << TRIG_PIN);   // HIGH
    _delay_us(10);
    
    PORTB &= ~(1 << TRIG_PIN);  // LOW
    
    // Wait for echo pulse to start (LOW to HIGH)
    uint32_t timeout = get_microseconds() + 30000;  // 30ms timeout
    while (!(PINB & (1 << ECHO_PIN))) {
        if (get_microseconds() > timeout) {
            return 0;  // Timeout
        }
    }
    
    // Measure pulse duration (HIGH to LOW)
    start_time = get_microseconds();
    timeout = start_time + 30000;  // 30ms timeout
    
    while (PINB & (1 << ECHO_PIN)) {
        if (get_microseconds() > timeout) {
            return 0;  // Timeout
        }
    }
    
    pulse_duration = get_microseconds() - start_time;
    
    // Calculate distance in cm (speed of sound = 343 m/s)
    distance = (pulse_duration * 343) / (2 * 10000);
    
    return distance;
}

// Send UART packet via software UART
void send_uart_packet(uint16_t distance) {
    uint8_t high_byte = (distance >> 8) & 0xFF;
    uint8_t low_byte = distance & 0xFF;
    
    soft_uart_transmit(START_BYTE);
    soft_uart_transmit(DISTANCE_CMD);
    soft_uart_transmit(high_byte);
    soft_uart_transmit(low_byte);
    soft_uart_transmit(END_BYTE);
}

// ==================== ARDUINO #2 - BARE METAL RECEIVER ====================

/*
 * Compile this separately for Arduino #2
 * 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pin definitions
#define LED_PIN PB5         // Pin 13 (built-in LED)
#define SOFT_RX_PIN PD2     // Pin 2 for receiving data
#define SOFT_TX_PIN PD3     // Pin 3 (not used in receiver)

// UART Protocol constants
#define START_BYTE 0xFF
#define DISTANCE_CMD 0x01
#define END_BYTE 0xFE

// Reception variables
volatile uint8_t rx_buffer[10];
volatile uint8_t rx_index = 0;
volatile uint8_t packet_ready = 0;

// Function prototypes
void init_receiver_system(void);
void init_hardware_uart(void);
void init_software_uart_rx(void);
void init_pin_change_interrupt(void);
void uart_transmit(uint8_t data);
uint8_t soft_uart_receive(void);
void process_packet(void);

int main(void) {
    init_receiver_system();
    
    // Send startup message
    const char* startup_msg = "Arduino #2 Ready - Receiving from Arduino #1\r\n";
    for (int i = 0; startup_msg[i] != '\0'; i++) {
        uart_transmit(startup_msg[i]);
    }
    
    while (1) {
        if (packet_ready) {
            process_packet();
            packet_ready = 0;
            rx_index = 0;
        }
        
        _delay_ms(100);
    }
    
    return 0;
}

// Initialize receiver system
void init_receiver_system(void) {
    cli();
    
    init_hardware_uart();
    init_software_uart_rx();
    init_pin_change_interrupt();
    
    // Set pin directions
    DDRB |= (1 << LED_PIN);     // LED pin as output
    DDRD &= ~(1 << SOFT_RX_PIN);// Software UART RX as input
    
    // Set initial states
    PORTB &= ~(1 << LED_PIN);   // LED OFF
    PORTD |= (1 << SOFT_RX_PIN);// Enable pull-up on RX pin
    
    sei();
}

// Initialize hardware UART (same as sender)
void init_hardware_uart(void) {
    uint16_t baud_setting = (F_CPU / (16UL * 9600)) - 1;
    
    UBRR0H = (uint8_t)(baud_setting >> 8);
    UBRR0L = (uint8_t)baud_setting;
    
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Initialize software UART reception
void init_software_uart_rx(void) {
    // Software UART reception will use pin change interrupt
    rx_index = 0;
    packet_ready = 0;
}

// Initialize pin change interrupt for software UART RX
void init_pin_change_interrupt(void) {
    // Enable pin change interrupt for PORTD
    PCICR |= (1 << PCIE2);
    
    // Enable pin change interrupt for PD2 (PCINT18)
    PCMSK2 |= (1 << PCINT18);
}

// Pin change interrupt for software UART reception
ISR(PCINT2_vect) {
    // Check if it's a falling edge on RX pin (start bit)
    if (!(PIND & (1 << SOFT_RX_PIN))) {
        // Disable pin change interrupt during reception
        PCMSK2 &= ~(1 << PCINT18);
        
        // Receive byte
        uint8_t received_byte = soft_uart_receive();
        
        // Store in buffer
        if (rx_index < 10) {
            rx_buffer[rx_index++] = received_byte;
            
            // Check for complete packet
            if (rx_index >= 5 && rx_buffer[0] == START_BYTE && 
                rx_buffer[4] == END_BYTE) {
                packet_ready = 1;
            }
        } else {
            rx_index = 0;  // Buffer overflow, reset
        }
        
        // Re-enable pin change interrupt
        PCMSK2 |= (1 << PCINT18);
    }
}

// Hardware UART transmit (same as sender)
void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Software UART receive
uint8_t soft_uart_receive(void) {
    uint8_t data = 0;
    uint8_t i;
    
    // Wait for middle of start bit
    _delay_us(52);  // Half bit time
    
    // Sample data bits
    for (i = 0; i < 8; i++) {
        _delay_us(104);  // One bit time
        
        if (PIND & (1 << SOFT_RX_PIN)) {
            data |= (1 << i);  // Set bit if HIGH
        }
    }
    
    // Wait for stop bit
    _delay_us(104);
    
    return data;
}

// Process received packet
void process_packet(void) {
    if (rx_buffer[0] == START_BYTE && rx_buffer[1] == DISTANCE_CMD && 
        rx_buffer[4] == END_BYTE) {
        
        uint16_t distance = (rx_buffer[2] << 8) | rx_buffer[3];
        
        // Send to Serial Monitor
        const char* msg1 = "Received distance: ";
        for (int i = 0; msg1[i] != '\0'; i++) {
            uart_transmit(msg1[i]);
        }
        
        char dist_str[10];
        itoa(distance, dist_str, 10);
        for (int i = 0; dist_str[i] != '\0'; i++) {
            uart_transmit(dist_str[i]);
        }
        
        const char* msg2 = " cm\r\n";
        for (int i = 0; msg2[i] != '\0'; i++) {
            uart_transmit(msg2[i]);
        }
        
        // Control LED based on distance
        if (distance < 20) {
            PORTB |= (1 << LED_PIN);   // LED ON
            const char* msg3 = "LED ON - Object close!\r\n";
            for (int i = 0; msg3[i] != '\0'; i++) {
                uart_transmit(msg3[i]);
            }
        } else {
            PORTB &= ~(1 << LED_PIN);  // LED OFF
            const char* msg4 = "LED OFF - Object far\r\n";
            for (int i = 0; msg4[i] != '\0'; i++) {
                uart_transmit(msg4[i]);
            }
        }
    }
}
