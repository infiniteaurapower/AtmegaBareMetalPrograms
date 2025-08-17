#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define RX_BUFFER_SIZE 64

volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // Enable RX, TX, and RX Complete Interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    sei(); // Enable global interrupts
}

void USART_Transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = data; // Put data into buffer, sends the data
}

void USART_Print(const char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
    USART_Transmit('\n');
}

unsigned char USART_Receive(void) {
    // Blocking receive
    while (rxHead == rxTail); // Wait for data
    unsigned char data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
    return data;
}

uint8_t USART_Available(void) {
    return (RX_BUFFER_SIZE + rxHead - rxTail) % RX_BUFFER_SIZE;
}

ISR(USART_RX_vect) {
    unsigned char data = UDR0; // Read received byte
    uint8_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
    if (nextHead != rxTail) { // Avoid buffer overflow
        rxBuffer[rxHead] = data;
        rxHead = nextHead;
    }
}

int main(void) {
    USART_Init(MYUBRR);

    USART_Print("UART Driver Ready!");

    while (1) {
        // If data is available, echo it back
        if (USART_Available()) {
            char c = USART_Receive();
            USART_Transmit(c); // Echo back
        }

        _delay_ms(100);
    }
}
