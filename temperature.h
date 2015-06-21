#include <Arduino.h>

// read out an am2302, don't call more then once per 2 seconds
void readAm2302(uint8_t pin, float* celcius, float* humidity);
