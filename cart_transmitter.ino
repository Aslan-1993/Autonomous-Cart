#define TX_PIN        11      // OC2A output to transistor base network
#define MODE_PIN       6      // Select: HIGH=150 Hz, LOW=300 Hz
#define MOD_DUTY_NUM   1      // 1/5 duty (≈20%)
#define MOD_DUTY_DEN   5

// --- Timer2: CTC toggle on OC2A -> square wave f = F_CPU/(2*N*(1+OCR2A))
// For 16 MHz, N=1, OCR2A=210-1 => ~37.93 kHz  (close to 38 kHz)
static inline void carrierSetup38kHz() {
  pinMode(TX_PIN, OUTPUT);
  // CTC mode (WGM21=1), toggle OC2A on compare (COM2A0=1), prescaler 1
  TCCR2A = _BV(WGM21);           // CTC, OC2A disconnected for now
  TCCR2B = _BV(CS20);            // prescaler = 1
  OCR2A  = 210 - 1;              // ≈ 37.93 kHz
}

static inline void carrierOn() {
  // Toggle OC2A on compare (connects timer to pin)
  TCCR2A |= _BV(COM2A0);
}

static inline void carrierOff() {
  // Disconnect timer from pin and force LOW
  TCCR2A &= ~_BV(COM2A0);
  digitalWrite(TX_PIN, LOW);
}

void setup() {
  pinMode(MODE_PIN, INPUT_PULLUP);   // slide switch to GND = 300 Hz, to VCC = 150 Hz
  carrierSetup38kHz();
  carrierOff();
}

void loop() {
  // Read modulation rate from switch (HIGH=150 Hz, LOW=300 Hz)
  uint16_t modHz = (digitalRead(MODE_PIN) == HIGH) ? 150 : 300;

  // Compute period and 20% mark time in microseconds
  uint32_t period_us = 1000000UL / modHz;                        // total period
  uint32_t mark_us   = (period_us * MOD_DUTY_NUM) / MOD_DUTY_DEN; // ≈20%
  uint32_t space_us  = period_us - mark_us;

  // One AM cycle: carrier ON for mark, OFF for space
  carrierOn();
  delayMicroseconds(mark_us);
  carrierOff();
  delayMicroseconds(space_us);
}
