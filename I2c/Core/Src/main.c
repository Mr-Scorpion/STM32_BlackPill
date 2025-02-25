#include "stm32f4xx.h"

// Define slave address and data buffer for transmission
#define SLAVE_ADDRESS 0x52  // 7-bit slave address
uint8_t dataBuffer[] = { 0xA5, 0x5A, 0xFF };  // Example data to transmit
#define DATA_COUNT (sizeof(dataBuffer)/sizeof(dataBuffer[0]))

// Simple delay function
void delay(volatile uint32_t count) {
    while(count--);
}

void I2C1_Init(void) {
    // 1. Enable clock for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;      // Enable I2C1 clock

    // 2. Configure GPIOB Pins 8 (SCL) and 9 (SDA) for Alternate Function
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);            // Clear mode
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);           // Alternate function
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                // Open-drain
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);              // Clear pull-up/down
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0);             // Pull-up
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos); // AF4 for I2C

    // 3. Software reset the I2C peripheral
    I2C1->CR1 |= I2C_CR1_SWRST;
    delay(1000);  // Short delay
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // 4. Configure I2C clock frequency
    I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ) | 16; // APB1 = 16 MHz

    // 5. Configure Clock Control Register (CCR) for 100 kHz standard mode
    // CCR = APB1_FREQ/(2 * I2C_FREQ)
    I2C1->CCR &= ~I2C_CCR_CCR;
    I2C1->CCR |= (uint32_t)(16e6 / (2 * 100e3));  // Explicitly cast to uint32_t

    // 6. Configure maximum rise time
    // For standard mode, TRISE = (maximum_rise_time_ns / APB1_period_ns) + 1
    // APB1 period = 1/(16MHz) ~ 62.5 ns, maximum rise time = 1000 ns
    I2C1->TRISE = (1000 / 62) + 1;  // (You can fine-tune this value)

    // 7. Set own address (for slave mode if needed)
    I2C1->OAR1 = (SLAVE_ADDRESS << 1);  // Set the 7-bit address shifted left by 1
    I2C1->OAR1 |= (1 << 15);             // Enable the own address by setting OA1EN (bit 15)

    // 8. Enable I2C peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Transmit(uint8_t slave_addr, uint8_t *data, uint32_t size) {
    uint32_t index = 0;

    // 1. Wait for the bus to be free (check BUSY bit)
    while (I2C1->SR2 & I2C_SR2_BUSY);

    // 2. Generate START condition
    I2C1->CR1 |= I2C_CR1_START;

    // 3. Wait until START condition is generated (check SB flag in SR1)
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // 4. Send slave address with write bit (LSB = 0)
    I2C1->DR = (slave_addr << 1);  // Write mode: LSB=0

    // 5. Wait until address is sent and ACK received (ADDR flag set)
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

    // 6. Clear ADDR flag by reading SR1 then SR2
    volatile uint32_t temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;  // Prevent compiler warning

    // 7. Transmit data bytes
    for (index = 0; index < size; index++) {
        // Write data byte to DR
        I2C1->DR = data[index];

        // Wait until TXE flag is set indicating DR is empty
        while (!(I2C1->SR1 & I2C_SR1_TXE));
    }

    // 8. Wait until the last byte transfer is complete (BTF flag)
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // 9. Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

int main(void) {
    // Initialize I2C1 peripheral
    I2C1_Init();

    // Transmit dataBuffer to the slave device with address SLAVE_ADDRESS
    I2C1_Transmit(SLAVE_ADDRESS, dataBuffer, DATA_COUNT);

    while (1) {
        // Main loop
    }
}
