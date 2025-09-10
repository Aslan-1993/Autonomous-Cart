#include <AFMotor.h>

// --------------------------- CONSTANTS ----------------------------
#define HC06        Serial2     // Arduino Mega: RX2(17), TX2(16)
const uint8_t SPEED = 100;      // 0..255 for AFMotor

// IR pins (digital input used with pulseIn frequency probe)
const uint8_t IRpinForward = 33;
const uint8_t IRpinRight   = 35;
const uint8_t IRpinLeft    = 31;

// Ultrasonic pins (HC-SR04)
const uint8_t trigLift      = 47;  // left Ultrasonic
const uint8_t echoLift      = 46;

const uint8_t trigMidRight  = 51; // middle right Ultrasonic
const uint8_t echoMidRight  = 50;

const uint8_t trigMidLeft   = 45; // middle left Ultrasonic
const uint8_t echoMidLeft   = 44;

const uint8_t trigRight     = 49; // right Ultrasonic
const uint8_t echoRight     = 48;

// Thresholds & timings
const int     IR_THRESH          = 150;   // “signal present” if > IR_THRESH
const uint8_t MID_GO_CM          = 35;    // safe corridor distance (mid sonars)
const uint8_t OBST_CLOSE0_CM     = 30;    // stop/turn distance modes 0/1
const uint8_t OBST_CLOSE3_CM     = 45;    // stop/turn distance mode 3
const uint8_t CLEAR3_CM          = 50;    // target clear distance in mode 3

const unsigned long PULSE_TIMEOUT_US = 80000UL; // 80 ms for pulseIn timeouts
const uint16_t SHORT_DELAY_MS  = 10;

// --------------------------- MOTORS -------------------------------
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// --------------------------- HELPERS: MOTION ----------------------
inline void m_setSpeed(uint8_t s) {
  motor1.setSpeed(s); motor2.setSpeed(s); motor3.setSpeed(s); motor4.setSpeed(s);
}
inline void forward() {
  motor1.run(FORWARD); motor2.run(FORWARD); motor3.run(FORWARD); motor4.run(FORWARD);
}
inline void left() {
  motor1.run(BACKWARD); motor2.run(FORWARD); motor3.run(FORWARD); motor4.run(BACKWARD);
}
inline void right() {
  motor1.run(FORWARD); motor2.run(BACKWARD); motor3.run(BACKWARD); motor4.run(FORWARD);
}
inline void Stop() {
  motor1.run(RELEASE); motor2.run(RELEASE); motor3.run(RELEASE); motor4.run(RELEASE);
}

// --------------------------- HELPERS: IO --------------------------
inline void trigPulse(uint8_t trigPin) {
  // HC-SR04 trigger: LOW 2 µs → HIGH 10 µs → LOW
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

int readSonarCM(uint8_t trigPin, uint8_t echoPin, uint8_t settleDelayMs = 15) {
  trigPulse(trigPin);
  unsigned long t = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US); // microseconds
  if (t == 0) return 0; // timeout/no echo
  // distance in cm (speed of sound ~0.034 cm/us, divide by 2 for round-trip)
  int cm = (int)((t * 0.034f) / 2.0f);
  delay(settleDelayMs); // brief settling to reduce cross-talk
  return cm;
}

// Returns a crude “carrier” proxy using high/low pulse period; safe on timeout
int readIRfreq(uint8_t pin) {
  unsigned long t1 = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  unsigned long t2 = pulseIn(pin, LOW,  PULSE_TIMEOUT_US);
  if (t1 == 0 || t2 == 0) return 0; // no signal, or timeout
  unsigned long period = t1 + t2;   // microseconds
  if (period == 0) return 0;
  // Frequency in Hz ≈ 1e6 / period
  return (int)(1000000UL / period);
}

// Non-blocking command poller; returns 0 if nothing new
char pollCmd() {
  if (HC06.available()) {
    return (char)HC06.read();
  }
  return 0;
}

// Check for immediate stop
bool checkStop(char c) {
  if (c == '4') {
    Stop();
    return true;
  }
  return false;
}

// --------------------------- SENSORS WRAPPERS ---------------------
inline int sonicMidLeft()  { return readSonarCM(trigMidLeft,  echoMidLeft,  15); }
inline int sonicMidRight() { return readSonarCM(trigMidRight, echoMidRight, 15); }
inline int sonicLift()     { return readSonarCM(trigLift,     echoLift,     20); }
inline int sonicRight()    { return readSonarCM(trigRight,    echoRight,    25); }

inline int irForward() { return readIRfreq(IRpinForward); }
inline int irRight()   { return readIRfreq(IRpinRight); }
inline int irLeft()    { return readIRfreq(IRpinLeft); }

// --------------------------- MODES --------------------------------
void mode0_leftBias() {
  // Autonomous with left fallback when no IR sees the path
  for (;;) {
    char c = pollCmd();
    if (checkStop(c)) return;
    if (c && c != '0') return; // new mode request

    if (irForward() > IR_THRESH) {
      forward();
      delay(SHORT_DELAY_MS);

      // Continue forward while corridor is clear and IR still sees path
      while ((sonicMidLeft() > MID_GO_CM || sonicMidRight() > MID_GO_CM) &&
             (irForward() > IR_THRESH)) {
        forward();
        if (checkStop(pollCmd())) return;
      }

      Stop();
      delay(5);

      if ((sonicMidLeft() < OBST_CLOSE0_CM || sonicMidRight() < OBST_CLOSE0_CM) &&
          (irLeft() > IR_THRESH || irRight() > IR_THRESH || irForward() > IR_THRESH)) {
        Stop();
        delay(500);
        return; // exit mode after gating condition
      }
    } else if (irRight() > IR_THRESH) {
      right(); delay(350);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) {
        right(); delay(200);
        Stop();  delay(100);
      }
    } else if (irLeft() > IR_THRESH) {
      left();  delay(300);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) {
        left();  delay(200);
        Stop();  delay(100);
      }
    } else {
      // None of the IRs see the line → left bias probe
      Stop();  delay(250);
      left();  delay(250);
      Stop();  delay(100);
    }
  }
}

void mode1_rightBias() {
  // Same as mode 0, but fallback turn is RIGHT
  for (;;) {
    char c = pollCmd();
    if (checkStop(c)) return;
    if (c && c != '1') return; // new mode

    if (irForward() > IR_THRESH) {
      forward();
      delay(SHORT_DELAY_MS);
      while ((sonicMidLeft() > MID_GO_CM || sonicMidRight() > MID_GO_CM) &&
             (irForward() > IR_THRESH)) {
        forward();
        if (checkStop(pollCmd())) return;
      }
      Stop(); delay(5);

      if ((sonicMidLeft() < OBST_CLOSE0_CM || sonicMidRight() < OBST_CLOSE0_CM) &&
          (irLeft() > IR_THRESH || irRight() > IR_THRESH || irForward() > IR_THRESH)) {
        Stop(); delay(500);
        return;
      }
    } else if (irRight() > IR_THRESH) {
      right(); delay(350);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) {
        right(); delay(200);
        Stop();  delay(100);
      }
    } else if (irLeft() > IR_THRESH) {
      left();  delay(300);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) {
        left();  delay(200);
        Stop();  delay(100);
      }
    } else {
      // Right bias probe
      Stop();  delay(250);
      right(); delay(250);
      Stop();  delay(100);
    }
  }
}

void mode3_obstacleCourse() {
  // More complex detour behavior using side sonars
  int flag   = 0;   // tracks detour phase
  int flagIR = 0;   // set when any IR sees a path

  for (;;) {
    char c = pollCmd();
    if (checkStop(c)) return;
    if (c && c != '3') return; // new mode

    // If IR sees path, behave like forward segment with gating
    if (irForward() > IR_THRESH) {
      flagIR = 1;
      forward(); delay(SHORT_DELAY_MS);

      while ((sonicMidLeft() > MID_GO_CM || sonicMidRight() > MID_GO_CM) &&
             (irForward() > IR_THRESH)) {
        forward();
        if (checkStop(pollCmd())) return;
      }

      Stop(); delay(5);

      if ((sonicMidLeft() < OBST_CLOSE3_CM || sonicMidRight() < OBST_CLOSE3_CM) &&
          (irLeft() > IR_THRESH || irRight() > IR_THRESH || irForward() > IR_THRESH)) {
        Stop(); delay(500);
        return;
      }
    } else if (irRight() > IR_THRESH) {
      flagIR = 1;
      right(); delay(350);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) { right(); delay(200); Stop(); delay(100); }
    } else if (irLeft() > IR_THRESH) {
      flagIR = 1;
      left();  delay(300);
      Stop();  delay(100);
      if (irForward() < IR_THRESH) { left(); delay(200); Stop(); delay(100); }
    } else {
      // No IR signal — only do this corridor logic if we haven't already used IR
      if (flagIR == 0) {
        // Advance until corridor narrows
        while ((sonicMidLeft() > OBST_CLOSE3_CM || sonicMidRight() > OBST_CLOSE3_CM) && (flag < 2)) {
          forward(); delay(SHORT_DELAY_MS);
          if (checkStop(pollCmd())) return;
        }

        if (sonicMidLeft() <= OBST_CLOSE3_CM || sonicMidRight() <= OBST_CLOSE3_CM) {
          Stop(); delay(500);
          if (flag == 0) {
            // Detour right until left side opens
            while (sonicLift() > OBST_CLOSE3_CM) {
              right();
              if (checkStop(pollCmd())) return;
            }
            right(); delay(30); Stop(); delay(500);

            // Move forward until left is comfortably clear
            while (sonicLift() < CLEAR3_CM) {
              forward();
              if (checkStop(pollCmd())) return;
            }
            Stop(); delay(500);
            left(); delay(600); Stop(); delay(500);

            forward(); delay(200); // re-enter corridor
            flag++;
          } else if (flag == 1) {
            // Detour left until right side opens
            while (sonicRight() > OBST_CLOSE3_CM) {
              left();
              if (checkStop(pollCmd())) return;
            }
            left(); delay(30); Stop(); delay(500);

            while (sonicRight() < CLEAR3_CM) {
              forward();
              if (checkStop(pollCmd())) return;
            }
            Stop(); delay(500);
            right(); delay(620); Stop(); delay(500);

            forward(); delay(100);
            flag++;
          }
          delay(50);
        }
      }
    }
  }
}

// --------------------------- ARDUINO CORE -------------------------
void setup() {
  Serial.begin(115200);
  HC06.begin(9600);

  // IR pins as inputs (using pulseIn)
  pinMode(IRpinForward, INPUT);
  pinMode(IRpinRight,   INPUT);
  pinMode(IRpinLeft,    INPUT);

  // Ultrasonic pins
  pinMode(trigLift,     OUTPUT); pinMode(echoLift,     INPUT);
  pinMode(trigMidLeft,  OUTPUT); pinMode(echoMidLeft,  INPUT);
  pinMode(trigMidRight, OUTPUT); pinMode(echoMidRight, INPUT);
  pinMode(trigRight,    OUTPUT); pinMode(echoRight,    INPUT);

  m_setSpeed(SPEED);
  Stop();

  Serial.println(F("Robot ready. Commands via HC-06: '0'=LeftBias, '1'=RightBias, '3'=ObstacleCourse, '4'=Stop"));
}

void loop() {
  // Idle until a command arrives
  char c = pollCmd();
  if (!c) return;

  Serial.print(F("CMD: ")); Serial.println(c);

  switch (c) {
    case '0': mode0_leftBias();      break;
    case '1': mode1_rightBias();     break;
    case '3': mode3_obstacleCourse();break;
    case '4': default: Stop();       break;
  }
}
