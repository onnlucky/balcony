// author: Onne Gorter
// license: CC0 http://creativecommons.org/publicdomain/zero/1.0/

#include "Time.h"

struct state {
    struct pump {
        int duration;
        bool on;
        time_t next_time;
        time_t last_time;
    } pump;
    struct highbucket {
        time_t last_time;
        int level;
    } highbucket;
    struct mainbucket {
        time_t last_time;
        int level;
    } mainbucket;
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

#define SSID F("SSID")
#define PASS F("PASS")
#define HOST F("192.168.87.101")
#define PORT 6979

// moist sensor in the high up bucket, used as "full" indicator
#define HIGHBUCKET_LEVEL_PIN A0
#define HIGHBUCKET_LEVEL_REACHED 800

// waterpump, pumps from main bucket into the high up bucket
#define PUMP_PIN 12
#define PUMP_MAX_SECONDS 60

// am2302 temperature sensor
#define TEMP_PIN 11

// capacitive water level sensor in the main bucket
#define MAINBUCKET_LEVEL_PIN1 7
#define MAINBUCKET_LEVEL_PIN2 6
#define MAINBUCKET_LEVEL_EMPTY 44
#define MAINBUCKET_LEVEL_FULL 65

// esp8266
#define ESP_RX 2
#define ESP_TX 3
#define ESP_RESET 4

#define REV 6

// -- end of config --

#include "capacity.h"
#include "temperature.h"
#include "esp8266.h"
#include <SoftwareSerial.h>
#include <avr/eeprom.h>

#define NEAR 10

SoftwareSerial esp(ESP_TX, ESP_RX);
ESP8266 wifi(esp, ESP_RESET);
uint8_t packetbuffer[32];

static inline uint8_t checksum(uint32_t from) {
    return (from >> 24) + (from >> 16) + (from >> 8) + from;
}

void setup() {
    esp.begin(9600);
    Serial.begin(9600);

    pinMode(ESP_RESET, INPUT);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    uint32_t last_time = eeprom_read_dword(0);
    uint8_t check = eeprom_read_byte((uint8_t*)4);
    if (checksum(last_time) == check) state.pump.last_time = last_time;

    Serial.print(F("balcony rev "));
    Serial.print(REV);
    Serial.print(F(" pump.last_time: "));
    char buf[20];
    snprintf(buf, sizeof(buf), "%02d:%02d", hour(state.pump.last_time), minute(state.pump.last_time));
    Serial.println(buf);
}

void measure_highbucket_level() {
    state.highbucket.level = analogRead(HIGHBUCKET_LEVEL_PIN);
    state.highbucket.last_time = now();
}

void measure_mainbucket_level() {
    state.mainbucket.level = readCapacity2(MAINBUCKET_LEVEL_PIN1, MAINBUCKET_LEVEL_PIN2);
    state.mainbucket.last_time = now();
}

#define midnight previousMidnight
#define hours(H) ((H) * SECS_PER_HOUR)
#define minutes(M) ((M) * SECS_PER_MIN)
void pump_schedule_next(time_t t) {
    state.pump.next_time = midnight(t) + hours(9) + minutes(15);

    if (state.pump.next_time < t) {
        // if last time was before 3am in the morning, now
        if (midnight(state.pump.last_time - hours(3)) <= midnight(t)) {
            state.pump.next_time = t;
        } else {
            // tomorrow
            state.pump.next_time += hours(24);
        }
    }
}

void pump_off() {
    state.pump.on = false;
    eeprom_write_dword(0, state.pump.last_time);
    eeprom_write_byte((uint8_t*)4, checksum(state.pump.last_time));
}

void pump_on(time_t t) {
    state.pump.on = true;
    state.pump.duration = 0;
    state.pump.last_time = t;
}

void service_pump() {
    uint32_t t = now();
    do {
        if (timeStatus() == timeNotSet) {
            state.pump.on = false;
            break;
        }

        if (state.pump.on) {
            state.pump.duration = t - state.pump.last_time;

            if (state.pump.duration > PUMP_MAX_SECONDS) {
                Serial.println(F("pump off: max time"));
                pump_off();
                break;
            }

            measure_highbucket_level();
            if (state.highbucket.level <= HIGHBUCKET_LEVEL_REACHED) {
                Serial.println(F("pump off: highbucket full"));
                pump_off();
                break;
            }

            measure_mainbucket_level();
            if (state.mainbucket.level >= 0 && state.mainbucket.level <= MAINBUCKET_LEVEL_EMPTY) {
                Serial.println(F("pump off: main bucket empty"));
                state.pump.on = false;
                break;
            }

            break;
        }

        if (state.pump.next_time <= state.pump.last_time) {
            pump_schedule_next(t);
            Serial.print(F("pump schedule next: "));
            Serial.println(state.pump.next_time);
        }

        if (state.pump.next_time < t) {
            Serial.println(F("pump on: time"));
            pump_on(t);
        }

    } while (false);

    digitalWrite(PUMP_PIN, state.pump.on);
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

void service_bucket() {
    if (state.highbucket.last_time + 5 > now()) return;
    measure_highbucket_level();
}

void service_main() {
    if (state.mainbucket.last_time + 5 > now()) return;
    measure_mainbucket_level();
}

bool start_wifi() {
    int s;
    if ((s = wifi.hardwareReset())) {
        Serial.print(F("wifi: reset error "));
        Serial.println(s);
        return false;
    }

    if ((s = wifi.getVersion((char*)packetbuffer, sizeof(packetbuffer)))) {
        Serial.print(F("wifi: version error "));
        Serial.println(s);
        return false;
    }
    Serial.print(F("wifi: esp8266 "));
    Serial.println((const char*)packetbuffer);

    if ((s = wifi.joinAP(SSID, PASS))) {
        Serial.print(F("wifi: join error "));
        Serial.println(s);
        return false;
    }

    if ((s = wifi.getIP((char*)packetbuffer, sizeof(packetbuffer)))) {
        Serial.print(F("wifi: ip error "));
        Serial.println(s);
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

    int s;
    wifi.putPacketBuffer(packetbuffer, sizeof(packetbuffer));
    if ((s = wifi.tcpOpen(HOST, PORT))) {
        Serial.print(F("wifi: tcp open error "));
        Serial.println(s);
        state.wifi.started = false;
        return false;
    }

    state.wifi.connected = true;
    return true;
}

void send_data() {
    char buf[200];

    time_t t = now();
    snprintf(buf, sizeof(buf), "%d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
    Serial.println(buf);

    static const char fmt[] PROGMEM =
        "{id=\"balcony\",r=%d,"
        "pump={duration=%d,next=%lu,last=%lu},"
        "highbucket={level=%d,last=%lu},"
        "mainbucket={level=%d,last=%lu},"
        "temperature={celcius=%s,humidity=%s,last=%lu}}\n";

    // %f format only works when adding this to gcc: -Wl,-u,vfprintf -lprintf_flt -lm
    char cbuf[14];
    char hbuf[14];
    dtostrf(state.temperature.celcius, 0, 2, cbuf);
    dtostrf(state.temperature.humidity, 0, 2, hbuf);

    int len = snprintf_P(buf, sizeof(buf), fmt,
        REV,
        state.pump.duration, state.pump.next_time, state.pump.last_time,
        state.highbucket.level, state.highbucket.last_time,
        state.mainbucket.level, state.highbucket.last_time,
        cbuf, hbuf, state.temperature.last_time);

    if ((unsigned)len >= sizeof(buf)) {
        Serial.print(F("wifi: error buffer too small: "));
        Serial.println(len);
        return;
    }

    Serial.print(buf);

    if (timeStatus() == timeNotSet) return;
    if (!state.wifi.connected) return;

    int s;
    if ((s = wifi.tcpSend((const uint8_t*)buf, len))) {
        Serial.print(F("wifi: error send "));
        Serial.println(s);
        state.wifi.started = false;
        state.wifi.connected = false;
    }
    state.wifi.last_time = now();
}

void service_wifi() {
    if (!state.wifi.connected) {
        if (!connect_wifi()) return;
    }

    int len = wifi.available();
    if (len < 0) {
        Serial.print(F("wifi: receive error "));
        Serial.println(len);
        state.wifi.started = state.wifi.connected = false;
        return;
    }

    if ((unsigned)len >= sizeof(packetbuffer)) {
        Serial.println(F("wifi: error, too much data received"));
        return;
    }

    if (len > 0) {
        char* buf = (char*)wifi.takePacketBuffer();
        // strip buf of whitespace and \n
        for (int i = 0; i < len; i++) if (buf[i] <= 32) buf[i] = 0;
        buf[len] = 0;

        if (buf[0] >= '1' && buf[0] <= '4') { // time command
            char* endp = 0;
            time_t t = strtoul(buf, &endp, 10);
            if (endp && endp > buf) {
                setTime(t);
                Serial.print(F("command: set time: "));
                Serial.println(now());
            } else {
                Serial.print(F("command: error setting time: "));
                Serial.println(buf);
            }
        } else if (!strcmp(buf, "pump")) {
            Serial.println(F("command: pump"));
            state.pump.next_time = now();
        } else if (!strcmp(buf, "stop")) {
            Serial.println(F("command: stop"));
            if (state.pump.on) {
                Serial.println(F("pump off: stop received"));
                pump_off();
            }
        } else if (!strcmp(buf, "clear")) {
            Serial.println(F("command: clear"));
            eeprom_write_dword(0, 0);
            eeprom_write_byte((uint8_t*)4, 0);
        } else {
            Serial.print(F("command: error, received: "));
            Serial.println(buf);
        }
        wifi.putPacketBuffer(packetbuffer, sizeof(packetbuffer));
    }
}

void loop() {
    // first, so measuring sensors etc can use serial buffer
    service_wifi();

    service_temperature();
    service_bucket();
    service_main();
    service_pump();

    static unsigned long last;
    if ((unsigned long)(millis() - last) > 5000UL) {
        last = millis();
        send_data();
    }
}

