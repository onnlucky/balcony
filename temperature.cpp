#include "temperature.h"

#define HIGH_COUNT 6
void readAm2302(uint8_t pin, float* celcius, float* humidity) {
    // the sensor will send us 5 bytes, last byte is checksum
    uint8_t data[5] = {0};

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(20);

    // here we need precise timing
    noInterrupts();
    digitalWrite(pin, HIGH);
    delayMicroseconds(40);
    pinMode(pin, INPUT);

    int sample = 0;
    uint8_t last = HIGH;
    for (int i = 0; sample < 40; i++) {
        int us = 0;
        // loop until a high/low or low/high transition
        while (digitalRead(pin) == last) {
            if (us >= 100) break;
            delayMicroseconds(1);
            us += 1;
        }
        last = !last;

        if (us >= 100) break; // too long, not a good read
        if (i < 4) continue; // ignore first 4 transitions
        if (last) continue; // ignore low/high transitions

        data[sample / 8] <<= 1;
        if (us > HIGH_COUNT) data[sample / 8] |= 1;
        sample++;
    }
    interrupts();

    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (sample != 40 || checksum != data[4]) {
        *celcius = NAN;
        *humidity = NAN;
        return;
    }

    // sample count and checksum are correct, process raw data
    int sign = data[2] & 0x80? -1 : 1;
    *celcius = sign * ((data[2] & 0x7F) * 256.0 + data[3]) / 10.0;
    *humidity = (data[0] * 256.0 + data[1]) / 10.0;
    return;
}
