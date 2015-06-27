// author: Onne Gorter
// license: CC0 http://creativecommons.org/publicdomain/zero/1.0/

#include "Time.h"

struct state {
    struct pump {
        time_t next_time;
        time_t last_time;
        int duration;
    } pump;
    struct bucket {
        time_t last_time;
        int level;
    } bucket;
    struct main {
        time_t last_time;
        int level;
    } main;
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
#define HOST F("192.168.87.101")
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
#define REV 4

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

bool wifi_error = false;

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

    Serial.print("balcony rev ");
    Serial.print(REV);
    Serial.print(" pump.last_time: ");
    char buf[20];
    snprintf(buf, sizeof(buf), "%02d:%02d", hour(state.pump.last_time), minute(state.pump.last_time));
    Serial.println(buf);
}

// will pump up water, until water level is reached
// TOOD either process wifi events, or make this cooperative
void pump() {
    state.pump.last_time = now();
    unsigned long start = millis();

    Serial.println(F("pump on"));
    while (true) {
        unsigned long ms = millis();

        if ((unsigned long)(ms - start) > MAX_PUMP_MS) {
            Serial.println(F("max pump ms"));
            break;
        }

        int mlast = 0;
        int mstate = analogRead(BUCKET_LEVEL_PIN);
        if (abs(mlast - mstate) > NEAR) {
            mlast = mstate;
            if (mstate < BUCKET_LEVEL_REACHED) {
                Serial.println(F("bucket full"));
                state.bucket.level = mstate;
                state.bucket.last_time = now();
                break;
            }
        }

        digitalWrite(13, HIGH);
        digitalWrite(PUMP_PIN, HIGH);
    }

    state.pump.duration = now() - state.pump.last_time;
    eeprom_write_dword(0, state.pump.last_time);
    eeprom_write_byte((uint8_t*)4, checksum(state.pump.last_time));

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

void service_bucket() {
    if (state.bucket.last_time + 5 > now()) return;
    state.bucket.level = analogRead(BUCKET_LEVEL_PIN);
    state.bucket.last_time = now();
}

void service_main() {
    if (state.main.last_time + 5 > now()) return;
    state.main.level = readCapacity2(MAIN_LEVEL_PIN1, MAIN_LEVEL_PIN2);
    state.main.last_time = now();
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
    char buf[200];

    time_t t = now();
    snprintf(buf, sizeof(buf), "%d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
    Serial.println(buf);

    static const char fmt[] PROGMEM =
        "{id=\"balcony\",r=%d,"
        "pump={duration=%d,next=%lu,last=%lu},"
        "bucket={level=%d,last=%lu},"
        "main={level=%d,last=%lu},"
        "temp={celcius=%s,humidity=%s,last=%lu}}\n";

    // %f format only works when adding this to gcc: -Wl,-u,vfprintf -lprintf_flt -lm
    char cbuf[14];
    char hbuf[14];
    dtostrf(state.temperature.celcius, 0, 2, cbuf);
    dtostrf(state.temperature.humidity, 0, 2, hbuf);

    int len = snprintf_P(buf, sizeof(buf), fmt,
        REV,
        state.pump.duration, state.pump.next_time, state.pump.last_time,
        state.bucket.level, state.bucket.last_time,
        state.main.level, state.bucket.last_time,
        cbuf, hbuf, state.temperature.last_time);

    if ((unsigned)len >= sizeof(buf)) {
        Serial.print("wifi: error buffer too small: ");
        Serial.println(len);
        return;
    }


    Serial.print(buf);

    if (timeStatus() == timeNotSet) return;
    if (!state.wifi.connected) return;

    if (!wifi.tcpSend((const uint8_t*)buf, len)) {
        Serial.println("wifi: error send");
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
        } else if (strcmp(buf, "pump")) {
            Serial.println(F("command: pump"));
        } else if (strcmp(buf, "clear")) {
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

