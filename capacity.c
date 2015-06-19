#include "capacity.h"

#define PIN_TO_OUTPUTREG(pin)             (portOutputRegister(digitalPinToPort(pin)))
#define PIN_TO_INPUTREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_MODEREG(pin)             (portModeRegister(digitalPinToPort(pin)))

#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask), (*((base)+2)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

unsigned int readCapacity1(int pin) {
    int total = 1;
    volatile uint8_t* outr = PIN_TO_OUTPUTREG(pin);
    volatile uint8_t* in_r = PIN_TO_INPUTREG(pin);
    volatile uint8_t* mode = PIN_TO_MODEREG(pin);
    const uint8_t mask = digitalPinToBitMask(pin);

    // discharge
    *outr &= ~(mask);
    *mode |= mask;
    delay(1);

    noInterrupts();
    *mode &= ~mask;
    *outr |= mask;
    while (!(*in_r & mask) && total) total++;
    interrupts();

    // discharge
    *outr &= ~mask;
    *mode |= mask;

    return total;
}

// 2 pin method
unsigned int readCapacity2(uint8_t pin1, uint8_t pin2) {
    int total = 1;
    volatile uint8_t* sReg = PIN_TO_INPUTREG(pin1);
    volatile uint8_t* rReg = PIN_TO_INPUTREG(pin2);
    const uint8_t sBit = digitalPinToBitMask(pin1);
    const uint8_t rBit = digitalPinToBitMask(pin2);

    // discharge
    DIRECT_WRITE_LOW(sReg, sBit);  // low
    DIRECT_MODE_INPUT(rReg, rBit); // pullups off
    DIRECT_MODE_OUTPUT(rReg, rBit);// as output
    DIRECT_WRITE_LOW(rReg, rBit);  // low
    delay(1);

    // charge and measure until total overflow
    noInterrupts();
    DIRECT_MODE_INPUT(rReg, rBit); // receive as input
    DIRECT_WRITE_HIGH(sReg, sBit); // send high
    while (!DIRECT_READ(rReg, rBit) && total) total++;
    interrupts();

    // discharge
    DIRECT_MODE_INPUT(rReg, rBit);
    DIRECT_WRITE_LOW(sReg, sBit);

    return total;
}
