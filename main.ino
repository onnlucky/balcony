#define NDEBUG 1
#define REV 2

#define SSID "SSID"
#define PASS "PASS"
#define HOST "192.168.1.1"
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

// -- end of config --

#include "capacity.h"
#include "temperature.h"
#include "ESP8266.h"

#include <SoftwareSerial.h>

#define NEAR 10
#define SECONDS_PER_DAY 86400L

#define STR(x) STR2(x)
#define STR2(x) #x
#define assert(x) do { if (!x) { \
Serial.println("Assertion Failed: " __FILE__ ":" STR(__LINE__) ": " #x); \
}} while (0)

struct state {
    struct time {
        bool set;
        unsigned long since_midnight;
        unsigned long last_update;
    } time;
    struct pump {
        unsigned long next_time;
        unsigned long last_time;
        int duration;
    } pump;
    struct temperature {
        unsigned long last_time;
        float celcius;
        float humidity;
    } temperature;
    struct comm {
        unsigned long last_time;
    } comm;
} state;

// time management

void setTime(unsigned long time) {
    state.time.since_midnight = time % SECONDS_PER_DAY;
    state.time.last_update = millis();
    state.time.set = true;
}

void updateTime() {
    unsigned long now = millis();
    unsigned long delta = now - state.time.last_update;
    if (delta >= 1000) {
        state.time.since_midnight += delta / 1000;
        if (state.time.since_midnight > SECONDS_PER_DAY) {
            state.time.since_midnight -= SECONDS_PER_DAY;
        }
        state.time.last_update = now / 1000 * 1000;
    }
}

unsigned long timeDelta(unsigned long t1, unsigned long t2) {
    return t1 - t2;
}

// returns true when t1 is after reference time
bool isAfter(unsigned long t1, unsigned long ref) {
    unsigned long delta = timeDelta(t1, ref);
    return delta != 0 && delta < 1209600000L; // 14 days
}

// get a future millis timestamp for a point in the day
// will always returns something between 0 - 24 hours from now
unsigned long millisAtTime(int hour, int minutes) {
    unsigned long seconds = (hour * 60L + minutes) * 60L;
    if (seconds < state.time.since_midnight) seconds += SECONDS_PER_DAY;

    unsigned long millis_till_now = state.time.since_midnight * 1000L;
    unsigned long now = millis();
    unsigned long future = now - millis_till_now + seconds * 1000L;

    assert(isAfter(future, now));
    return future;
}

SoftwareSerial esp(ESP_TX, ESP_RX);
ESP8266 wifi(esp);

void restartWifi() {
    delay(1000);
    wifi.restart();
    delay(1000);

    Serial.print("wifi: esp8266 ");
    Serial.println(wifi.getVersion().c_str());

    if (wifi.joinAP(SSID, PASS)) {
        Serial.print("wifi: ip ");
        Serial.println(wifi.getLocalIP().c_str());
    } else {
        Serial.println("wifi: join error");
    }
    wifi.disableMUX();
}

void setup() {
    Serial.begin(9600);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    Serial.print("balcony ");
    Serial.println(REV);

    restartWifi();
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

    state.pump.last_time = start;
    state.pump.duration = millis() - start;

    Serial.println("pump off");
    digitalWrite(13, LOW);
    digitalWrite(PUMP_PIN, LOW);
}

void service_pump() {
    if (!state.time.set) return;

    if (!isAfter(state.pump.next_time, state.pump.last_time)) {
        state.pump.next_time = millisAtTime(10, 15);
        assert(isAfter(state.pump.next_time, state.pump.last_time));
    }

    if (isAfter(millis(), state.pump.next_time)) {
        pump();
        state.pump.next_time = millisAtTime(10, 15);
        assert(!isAfter(state.pump.next_time, state.pump.last_time));
    }
}

void service_temperature() {
    if (timeDelta(millis(), state.temperature.last_time) < 5000L) return;

    float celcius;
    float humidity;
    readAm2302(TEMP_PIN, &celcius, &humidity);

    if (celcius && humidity) {
        state.temperature.celcius = celcius;
        state.temperature.humidity = humidity;
        state.temperature.last_time = millis();
    }
}

void service_data() {
    //if (timeDelta(millis(), state.temperature.last_time) < 60000L) return;

    if (!wifi.createTCP(HOST, PORT)) {
        Serial.println("wifi: tcp create error");
        delay(1000);
        restartWifi();
        return;
    }

    char buf[16];
    String data;

    data += "{id=\"balcony\",pump={dur=";
    data += state.pump.duration;
    data += ",last=";
    data += state.pump.last_time / 1000L;
    data += ",next=";
    data += state.pump.next_time / 1000L;

    data += "},temp={last=";
    data += state.temperature.last_time / 1000L;
    data += ",celcius=",
    dtostrf(state.temperature.celcius, 0, 2, buf); data += buf;
    data += ",humidity=",
    dtostrf(state.temperature.humidity, 0, 2, buf); data += buf;
    data += "}}";

    wifi.send((const uint8_t*)data.c_str(), data.length());

    int len = wifi.recv((uint8_t*)buf, sizeof(buf), 5000);
    if (len > 0) {
        int hour = (buf[0] - '0') * 10 + buf[1] - '0';
        int minute = (buf[2] - '0') * 10 + buf[3] - '0';
        if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60) {
            setTime((hour * 60L + minute) * 60L);
        }
    }

    if (!wifi.releaseTCP()) {
        Serial.println("wifi: tcp release error");
        delay(1000);
        restartWifi();
        return;
    }
}

void loop() {
    // read time from serial in HHMM
    if (Serial.available() >= 4) {
        int hour = (Serial.read() - '0') * 10 + Serial.read() - '0';
        int minute = (Serial.read() - '0') * 10 + Serial.read() - '0';
        if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60) {
            setTime((hour * 60L + minute) * 60L);
        }
    }

    static unsigned long last = 0;
    if (timeDelta(millis(), last) > 5000L) {
        last = millis();
        int hour = state.time.since_midnight / 3600L;
        int minute = state.time.since_midnight / 60L % 60L;
        Serial.print(hour / 10);
        Serial.print(hour % 10);
        Serial.print(minute / 10);
        Serial.print(minute % 10);
        Serial.print(" ");
        Serial.println(last);

        Serial.print("pump: ");
        Serial.print(state.pump.duration);
        Serial.print(" next: ");
        Serial.print(timeDelta(state.pump.next_time, last) / 1000L);
        Serial.print(" last: ");
        Serial.println(timeDelta(last, state.pump.last_time) / 1000L);

        Serial.print("temperature: ");
        Serial.print(state.temperature.celcius);
        Serial.print(" humidity: ");
        Serial.print(state.temperature.humidity);
        Serial.print(" last: ");
        Serial.println(timeDelta(last, state.temperature.last_time) / 1000L);
    }

    updateTime();
    service_pump();
    service_temperature();
    service_data();
}

