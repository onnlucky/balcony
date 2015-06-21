#define NDEBUG 1

#define SSID "SSID"
#define PASS "PASS"

// high up moist sensor, used as "full" indicator
#define BUCKET_LEVEL_PIN A0
#define BUCKET_LEVEL_REACHED 800

// waterpump
#define PUMP_PIN 12
#define MAX_PUMP_MS ((unsigned long)1000 * 60)

// am2302 temperature sensor
#define TEMP_PIN 11

// capacitive main water level sensor
#define MAIN_LEVEL_PIN1 7
#define MAIN_LEVEL_PIN2 6
#define MAIN_LEVEL_EMPTY 33
#define MAIN_LEVEL_FULL 80

// esp8266
#define ESP_RX 2
#define ESP_TX 3

// -- end of config --

#include "capacity.h"
#include "temperature.h"
#include <SoftwareSerial.h>

#define NEAR 10

SoftwareSerial esp(ESP_TX, ESP_RX);

void setup() {
    Serial.begin(9600);
    esp.begin(9600);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

// will pump up water, until water level is reached
void pump() {
    unsigned long start = millis();

    Serial.println("pump on");
    while (true) {
        unsigned long now = millis();

        if ((unsigned long)(now - start) > MAX_PUMP_MS) {
            Serial.println("max pump ms");
            break;
        }

        int mlast = 0;
        int mstate = analogRead(BUCKET_LEVEL_PIN);
        if (abs(mlast - mstate) > NEAR) {
            mlast = mstate;
            if (mstate < BUCKET_LEVEL_REACHED) {
                Serial.println("bucket full");
                break;
            }
        }

        digitalWrite(13, HIGH);
        digitalWrite(PUMP_PIN, HIGH);
    }

    Serial.println("pump off");
    digitalWrite(13, LOW);
    digitalWrite(PUMP_PIN, LOW);
}

void echo(int millis) {
    for (int i = 0; i < millis; i++) {
        if (esp.available()) Serial.write(esp.read());
        else delay(1);
    }
}

bool resetComm() {
    esp.println("AT+RST");
    echo(1600);
    esp.println("AT+CWMODE=1");
    echo(100);
    esp.println("AT+CWJAP=\"" SSID "\",\"" PASS "\"");
    echo(9000);
    esp.println("AT+CIPMUX=0");
    echo(100);
    esp.println("AT+CIPMODE=0");
    echo(100);
    return true;
}

bool communicate() {
    Serial.println("comm init ...");
    esp.flush();
    esp.println("AT+CIPSTART=\"TCP\",\"192.168.87.31\",9999");
    echo(2000);
    esp.println("AT+CIPSEND=4");
    echo(200);
    esp.print("helo");
    echo(5000);
    esp.println("AT+CIPCLOSE");
    echo(2000);
    return true;
}

void loop() {
    float celcius;
    float humidity;
    readAm2302(TEMP_PIN, &celcius, &humidity);

    Serial.print("temperature: ");
    Serial.print(celcius);
    Serial.print(" humidity: ");
    Serial.print(humidity);
    Serial.println("");

    delay(5000);
/*


    pump();

    delay(1000);
    resetComm();
    while (true) {
        delay(1000);
        communicate();
        delay(5000);
    }
*/
}

