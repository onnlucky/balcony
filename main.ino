// author: Onne Gorter
// license: CC0 http://creativecommons.org/publicdomain/zero/1.0/

#include "Time.h"

struct state {
    struct pump {
        time_t next_time;
        time_t last_time;
        int duration;
    } pump;
    struct temperature {
        time_t last_time;
        float celcius;
        float humidity;
    } temperature;
    struct wifi {
        time_t last_time;
        bool started;
        bool connected;
    } wifi;
} state;

#define SSID "SSID"
#define PASS "PASS"
#define HOST "192.168.87.101"
#define PORT 6979

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
#define ESP_RESET 4

#define NDEBUG 1
#define REV 3

// -- end of config --

#include "capacity.h"
#include "temperature.h"
#include "esp8266.h"
#include <SoftwareSerial.h>

#define NEAR 10

SoftwareSerial esp(ESP_TX, ESP_RX);
ESP8266 wifi(esp, ESP_RESET);
uint8_t packetbuffer[32];

bool wifi_error = false;

void setup() {
    esp.begin(9600);
    Serial.begin(9600);

    pinMode(ESP_RESET, INPUT);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    Serial.print("balcony rev ");
    Serial.println(REV);
}

// will pump up water, until water level is reached
void pump() {
    state.pump.last_time = now();
    unsigned long start = millis();

    Serial.println(F("pump on"));
    while (true) {
        unsigned long now = millis();

        if ((unsigned long)(now - start) > MAX_PUMP_MS) {
            Serial.println(F("max pump ms"));
            break;
        }

        int mlast = 0;
        int mstate = analogRead(BUCKET_LEVEL_PIN);
        if (abs(mlast - mstate) > NEAR) {
            mlast = mstate;
            if (mstate < BUCKET_LEVEL_REACHED) {
                Serial.println(F("bucket full"));
                break;
            }
        }

        digitalWrite(13, HIGH);
        digitalWrite(PUMP_PIN, HIGH);
    }

    state.pump.duration = now() - state.pump.last_time;

    Serial.println(F("pump off"));
    digitalWrite(13, LOW);
    digitalWrite(PUMP_PIN, LOW);
}

#define midnight previousMidnight
#define hours(H) ((H) * SECS_PER_HOUR)
#define minutes(M) ((M) * SECS_PER_MIN)
void service_pump() {
    if (timeStatus() == timeNotSet) return;

    if (state.pump.next_time <= state.pump.last_time) {
        state.pump.next_time = midnight(now()) + hours(10) + minutes(15);
        if (state.pump.next_time < now()) state.pump.next_time += hours(24);
    }

    if (state.pump.next_time < now()) pump();
}

void service_temperature() {
    if (state.temperature.last_time + 5 > now()) return;

    float celcius;
    float humidity;
    readAm2302(TEMP_PIN, &celcius, &humidity);

    if (celcius && humidity) {
        state.temperature.celcius = celcius;
        state.temperature.humidity = humidity;
        state.temperature.last_time = now();
    }
}

bool start_wifi() {
    if (!wifi.hardwareReset()) {
        Serial.println(F("wifi: reset error"));
        return false;
    }

    if (!wifi.getVersion((char*)packetbuffer, sizeof(packetbuffer))) {
        Serial.println(F("wifi: version error"));
        return false;
    }
    Serial.print(F("wifi: esp8266 "));
    Serial.println((const char*)packetbuffer);

    if (!wifi.joinAP(SSID, PASS)) {
        Serial.println(F("wifi: join error"));
        return false;
    }

    if (!wifi.getIP((char*)packetbuffer, sizeof(packetbuffer))) {
        Serial.println(F("wifi: ip error"));
        return false;
    }
    Serial.print(F("wifi: ip "));
    Serial.println((const char*)packetbuffer);

    state.wifi.started = true;
    return true;
}

bool connect_wifi() {
    if (!state.wifi.started) {
        if (!start_wifi()) return false;
    }

    wifi.putPacketBuffer(packetbuffer, sizeof(packetbuffer));
    if (!wifi.tcpOpen(HOST, PORT)) {
        Serial.println(F("wifi: tcp open error"));
        state.wifi.started = false;
        return false;
    }

    state.wifi.connected = true;
    return true;
}

void send_data() {
    char buf[20];
    String data;
    data += "{id=\"balcony\",pump={dur=";
    data += state.pump.duration;
    data += ",last=";
    data += state.pump.last_time;
    data += ",next=";
    data += state.pump.next_time;

    data += "},temp={last=";
    data += state.temperature.last_time;
    data += ",celcius=",
    dtostrf(state.temperature.celcius, 0, 2, buf); data += buf;
    data += ",humidity=",
    dtostrf(state.temperature.humidity, 0, 2, buf); data += buf;
    data += "}}\n";

    if (!wifi.tcpSend((const uint8_t*)data.c_str(), data.length())) {
        Serial.println("wifi: error send");
        state.wifi.started = false;
        state.wifi.connected = false;
    }
    state.wifi.last_time = now();
}

void service_wifi() {
    if (state.wifi.last_time + 5 > now()) return;

    if (!state.wifi.connected) {
        if (!connect_wifi()) return;
    }

    int len = wifi.available();
    if (len < 0) {
        state.wifi.started = state.wifi.connected = false;
        return;
    }
    if (len > 0) {
        char* buf = (char*)wifi.takePacketBuffer();
        buf[len] = 0;
        char* endp = 0;
        time_t t = strtoul(buf, &endp, 10);
        if (endp && endp > buf) {
            setTime(t);
            Serial.print(F("set time: "));
            Serial.println(now());
        } else {
            Serial.print(F("received: "));
            Serial.println(buf);
        }
        wifi.putPacketBuffer(packetbuffer, sizeof(packetbuffer));
    }

    if (timeStatus() != timeNotSet) send_data();
}

void loop() {
    static unsigned long last = -5001UL;
    if ((unsigned long)(millis() - last) > 5000UL) {
        last = millis();
        time_t t = now();
        char buf[20];
        snprintf(buf, sizeof(buf), "%d-%02d-%02dT%02d:%02d:%02d",
            year(t), month(t), day(t), hour(t), minute(t), second(t));
        Serial.println(buf);

        Serial.print(F("pump: "));
        Serial.print(state.pump.duration);
        Serial.print(F(" next: "));
        snprintf(buf, sizeof(buf), "%02d:%02d",
            hour(state.pump.next_time), minute(state.pump.next_time));
        Serial.print(buf);
        Serial.print(F(" last: "));
        snprintf(buf, sizeof(buf), "%02d:%02d",
            hour(state.pump.last_time), minute(state.pump.last_time));
        Serial.println(buf);

        Serial.print(F("temperature: "));
        Serial.print(state.temperature.celcius);
        Serial.print(F(" humidity: "));
        Serial.print(state.temperature.humidity);
        Serial.print(F(" last: "));
        snprintf(buf, sizeof(buf), "%02d:%02d",
            hour(state.temperature.last_time), minute(state.temperature.last_time));
        Serial.println(buf);
    }

    service_temperature();
    service_pump();
    service_wifi();
}

