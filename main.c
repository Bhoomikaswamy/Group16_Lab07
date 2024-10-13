#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define SW1    (1U << 0)  // PF0 (SW1)
#define SW2    (1U << 4)  // PF4 (SW2)
#define RED_LED    (1U << 1) // PF1
#define BLUE_LED   (1U << 2) // PF2
#define GREEN_LED  (1U << 3) // PF3

uint8_t receivedByte;
bool dataReceivedFlag = true;

void Uart3_send(void);
void UART3_Transmit(uint8_t data);
void PortF_Initialisation(void);
void PortC_Initialisation(void);
void Uart3_Initialisation(void);
uint8_t UART3_ReceiveByte(void);
void Uart3_Read(void);

void PortF_Initialisation(void) {
    // Enable clock for Port F
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
    // Unlock PF0 (SW1)
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R = 0x1F; // Enable changes to PF0-PF4
    GPIO_PORTF_DEN_R = 0x1F; // Enable digital function
    GPIO_PORTF_DIR_R = 0x0E; // Set PF1-PF3 as output
    GPIO_PORTF_PUR_R = 0x11; // Enable pull-up resistors for SW1 (PF0) and SW2 (PF4)
    GPIO_PORTF_DATA_R = 0x00; // Initialize LEDs to off
}

void PortC_Initialisation(void) {
    // Enable the clock for UART3 and GPIO Port C
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOC;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;
    while ((SYSCTL_PRUART_R & SYSCTL_PRUART_R3) == 0) {}; // Wait for UART3 to be ready

    // Configure PC6 (RX) and PC7 (TX)
    GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTC_CR_R = 0xC0; // Enable changes to PC6-PC7
    GPIO_PORTC_DEN_R = 0xC0; // Enable digital function for PC6 and PC7
    GPIO_PORTC_AFSEL_R = 0xC0; // Enable alternate function for PC6 and PC7
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & ~0xFF000000) | 0x11000000; // Set PC6, PC7 to UART
}

void Uart3_Initialisation(void) {
    // Configure UART3 for 9600 baud rate, 8 data bits, odd parity
    UART3_CTL_R = 0x00; // Disable UART for config
    UART3_IBRD_R = 104; // 9600 baud rate: 16 MHz / (16 * 9600) = 104
    UART3_FBRD_R = 11;  // Fraction part of BRD
    UART3_LCRH_R = 0x72; // 8 bits, odd parity
    UART3_CC_R = 0x00;  // Use system clock
    UART3_CTL_R = 0x301; // Enable UART, Tx, and Rx
}

uint8_t UART3_ReceiveByte(void) {
    while ((UART3_FR_R & 0x10) != 0); // Wait for data
    return UART3_DR_R; // Return received data
}

void Uart3_Read(void) {
    receivedByte = UART3_ReceiveByte(); // Read data
    if (dataReceivedFlag) {
        // Check if a parity error occurred
        if (UART3_FR_R & 0x04) { // Parity Error
            GPIO_PORTF_DATA_R &= ~GREEN_LED; // Turn off Green
            GPIO_PORTF_DATA_R &= ~BLUE_LED;  // Turn off Blue
            GPIO_PORTF_DATA_R |= RED_LED;    // Turn on Red
        } else {
            switch (receivedByte) {
                case 0xAA:
                    GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn on Green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;  // Turn off Blue
                    GPIO_PORTF_DATA_R &= ~RED_LED;   // Turn off Red
                    break;
                case 0xF0:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED; // Turn off Green
                    GPIO_PORTF_DATA_R |= BLUE_LED;   // Turn on Blue LED
                    GPIO_PORTF_DATA_R &= ~RED_LED;   // Turn off Red
                    break;
                default:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED; // Turn off Green
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;  // Turn off Blue
                    GPIO_PORTF_DATA_R |= RED_LED;    // Turn on Red
                    break;
            }
        }
    }
}

void Uart3_send(void) {
    if (!(GPIO_PORTF_DATA_R & SW1)) { // SW1 pressed
        UART3_Transmit(0xF0);         // Transmit 0xF0
        while (!(GPIO_PORTF_DATA_R & SW1)); // Wait for release
    }
    if (!(GPIO_PORTF_DATA_R & SW2)) { // SW2 pressed
        UART3_Transmit(0xAA);         // Transmit 0xAA
        while (!(GPIO_PORTF_DATA_R & SW2)); // Wait for release
    }
}

void UART3_Transmit(uint8_t data) {
    while (UART3_FR_R & UART_FR_TXFF); // Wait if TX is full
    UART3_DR_R = data;                 // Send data
}

int main(void) {
    PortF_Initialisation(); // Initialize Port F
    PortC_Initialisation(); // Initialize Port C
    Uart3_Initialisation(); // Initialize UART3

    while (1) {
        Uart3_send(); // Check for switch press and transmit
        Uart3_Read(); // Check for incoming data and control LEDs
    }
}
