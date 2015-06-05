const int motor = 13;
const int button = 11;

#define DEBOUNCE_MS 30

#ifdef NDEBUG
// 10 minutes max in production mode
#define MAX_MOTOR_MS 1000 * 60 * 10
#else
// ten seconds max in debug mode
#define MAX_MOTOR_MS 1000 * 10
#endif

int done = false;

void setup() {
    Serial.begin(9600);
    Serial.println("hello");
    pinMode(motor, OUTPUT);
    pinMode(button, INPUT);
}

void loop() {
    // TODO deep sleep?
    if (done) {
        Serial.println("motor off");
        digitalWrite(motor, LOW);
        delay(5000);
        return;
    }

    long now = millis();
    if (now >= MAX_MOTOR_MS) {
        Serial.println("max motor ms");
        done = true;
        return;
    }

    static int blast = 0;
    static long btime = 0;
    int bstate = digitalRead(button);
    if (bstate != blast) {
        blast = bstate;
        btime = now;
    }
    if (btime && now - btime > DEBOUNCE_MS) {
        btime = 0;
        if (bstate) {
            Serial.println("button down");
            done = true;
            return;
        } else {
            Serial.println("button up");
        }
    }

    digitalWrite(motor, HIGH);
}

