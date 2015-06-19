#include <Arduino.h>

// to read miniscule capacities, like touching fingers, or fluid levels

// single pin method to read capacitance
// make sure pin can never short with HIGH
// pin -- capacitor -- gnd
// uses the ~10MOhm internal resistor of arduino
// likely reads 1 - 10
unsigned int readCapacity1(int pin);

// two pin method to read capacitance
// pin1 -+-- 1MOhm -- pin2
//       +-- capacitor -- gnd
// likely reads 10 - 100
unsigned int readCapacity2(uint8_t pin1, uint8_t pin2);
