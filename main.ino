#define NDEBUG 1

const int motor = 12;
const int button = 11;
const int moist = A0;

// anything below this means water is felt
#define WATER 800

#ifdef NDEBUG
// 30 seconds max in production mode
#define MAX_MOTOR_MS ((unsigned long)1000 * 30)
#else
// ten seconds max in debug mode
#define MAX_MOTOR_MS ((unsigned long)1000 * 10)
#endif

#define DEBOUNCE_MS 30
#define NEAR 10

int done = false;

void setup() {
    Serial.begin(9600);
    Serial.println("hello");
#ifndef NDEBUG
    Serial.println("debug version");
#endif
    pinMode(motor, OUTPUT);
    pinMode(button, INPUT);
}

void loop() {
    static int mlast = 0;
    int mstate = analogRead(moist);
    if (abs(mlast - mstate) > NEAR && mstate < 900) {
        mlast = mstate;
        Serial.print("moist: ");
        Serial.println(mstate, DEC);
        if (mstate < WATER) {
            Serial.println("under water");
            done = true;
        }
    }

    // TODO deep sleep?
    if (done) {
        Serial.println("motor off");
        digitalWrite(motor, LOW);
        delay(5000);
        return;
    }

    unsigned long now = millis();
    if (now >= MAX_MOTOR_MS) {
        Serial.println("max motor ms");
        done = true;
    }

    static int blast = 0;
    static unsigned long btime = 0;
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
        } else {
            Serial.println("button up");
        }
    }

    digitalWrite(motor, HIGH);
}

