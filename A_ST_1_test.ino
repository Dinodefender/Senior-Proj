// --- forward declare to avoid Arduino auto-prototype issues ---
struct AxisCmd;

/*
  Dual-Axis Solar Tracker — NC Interlocks + Active-HIGH sensing
  --------------------------------------------------------------
  Hardware:
    - L298N with ENA/ENB jumpers ON (full enable)
    - AZ motor: RED->OUT4, BLACK->OUT3 (controlled by D3->IN4, D4->IN3)
    - EL motor: RED->OUT1, BLACK->OUT2 (controlled by D8->IN1, D7->IN2)
    - LIMITS: NC contacts inline with the direction signal they guard,
              plus separate sensing lines that read HIGH at limit.
      AZ_MIN (D9)  guards AZ CCW  (D4->IN3)
      AZ_MAX (D10) guards AZ CW   (D3->IN4)
      EL_MIN (D11) guards EL DOWN (D7->IN2)
      EL_MAX (D12) guards EL UP   (D8->IN1)
    - Manual overrides (digital with external 10k pulldowns): A0=UP, A1=DOWN, A2=LEFT, A3=RIGHT

  Behavior:
    - Sensing remains ACTIVE-HIGH (LOW free, HIGH at stop).
    - Software never attempts to drive into a sensed HIGH.
    - On limit: coast (NC already cut), then back off a touch using the opposite line.
    - Homing uses short pulses toward the target stop, coasts on trip, backs off, and reseats gently.
*/

//////////////////// Pin map (matches your wiring) ////////////////////
#define AZ_IN1 4   // L298N IN3  (AZ CCW / LEFT)
#define AZ_IN2 3   // L298N IN4  (AZ CW  / RIGHT)
#define EL_IN1 8   // L298N IN1  (EL UP)1
#define EL_IN2 7   // L298N IN2  (EL DOWN)

// Limit sense inputs (ACTIVE-HIGH at stop)
#define AZ_MIN 9
#define AZ_MAX 10
#define EL_MIN 11
#define EL_MAX 12

// Manual override inputs (digital, need 10k pulldown to GND)
#define PIN_UP     A0  // EL UP
#define PIN_DOWN   A1  // EL DOWN
#define PIN_LEFT   A2  // AZ LEFT (CCW)
#define PIN_RIGHT  A3  // AZ RIGHT (CW)

enum Dir { STOP=0, FWD=1, REV=-1 }; // FWD: AZ CW / EL UP, REV: AZ CCW / EL DOWN

// Homing pulse timing
const uint8_t  FAST_ON_MS   = 50;
const uint8_t  FAST_OFF_MS  = 10;
const uint8_t  CREEP_ON_MS  = 8;
const uint8_t  CREEP_OFF_MS = 20;
const uint16_t BACKOFF_MS   = 180;
const unsigned long PHASE_TIMEOUT = 12000;

// Limit filtering (debounce) — still useful even with NC interlocks
const uint16_t LIMIT_FILTER_MS = 20;

//////////////////// Helpers ////////////////////
inline bool pinHigh(int p){ return digitalRead(p)==HIGH; }

bool heldHigh_ms(int pin, uint16_t ms){
  unsigned long t0 = millis();
  while (millis()-t0 < ms){
    if (digitalRead(pin)==LOW) return false;
    delay(2);
  }
  return true;
}
bool limitTripped(int pin){
  if (digitalRead(pin)==HIGH) return heldHigh_ms(pin, LIMIT_FILTER_MS);
  return false;
}

// Motor output primitives (no fast-brake; NC may open one line)
// AZ mapping: CW (RIGHT) uses D3->IN4 HIGH; CCW (LEFT) uses D4->IN3 HIGH
void coastAZ(){ digitalWrite(AZ_IN1,LOW); digitalWrite(AZ_IN2,LOW); }
void driveAZ(Dir d){
  // Guard with sensing (though NC already blocks the line)
  if (d==FWD && limitTripped(AZ_MAX)) { coastAZ(); return; } // CW blocked
  if (d==REV && limitTripped(AZ_MIN)) { coastAZ(); return; } // CCW blocked

  if (d==FWD){  // CW / RIGHT
    digitalWrite(AZ_IN1,LOW);
    digitalWrite(AZ_IN2,HIGH);
  } else if (d==REV){ // CCW / LEFT
    digitalWrite(AZ_IN1,HIGH);
    digitalWrite(AZ_IN2,LOW);
  } else {
    coastAZ();
  }
}

// EL mapping: UP uses D8->IN1 HIGH; DOWN uses D7->IN2 HIGH
void coastEL(){ digitalWrite(EL_IN1,LOW); digitalWrite(EL_IN2,LOW); }
void driveEL(Dir d){
  if (d==FWD && limitTripped(EL_MAX)) { coastEL(); return; } // UP blocked
  if (d==REV && limitTripped(EL_MIN)) { coastEL(); return; } // DOWN blocked

  if (d==FWD){  // UP
    digitalWrite(EL_IN1,HIGH);
    digitalWrite(EL_IN2,LOW);
  } else if (d==REV){ // DOWN
    digitalWrite(EL_IN1,LOW);
    digitalWrite(EL_IN2,HIGH);
  } else {
    coastEL();
  }
}

void stopAll(){ coastAZ(); coastEL(); }

//////////////////// Homing (NC aware) ////////////////////
// Generic pulse-toward-limit for AZ
bool pulseToLimit_AZ(Dir toward, int wantPin, int otherPin, int on_ms, int off_ms){
  unsigned long t0 = millis();
  while (millis()-t0 < PHASE_TIMEOUT){
    // If already at limit, we're done
    if (limitTripped(wantPin)) { coastAZ(); return true; }
    // If wrong stop is HIGH, wiring/dir issue
    if (limitTripped(otherPin)){ coastAZ(); Serial.println(F("[HOMING] AZ wrong limit HIGH.")); return false; }

    // Short pulse
    driveAZ(toward);
    delay(on_ms);
    coastAZ();
    delay(off_ms);

    // Did we hit the stop?
    if (limitTripped(wantPin)) { coastAZ(); return true; }
  }
  coastAZ();
  Serial.println(F("[HOMING] AZ timeout."));
  return false;
}

// Generic pulse-toward-limit for EL
bool pulseToLimit_EL(Dir toward, int wantPin, int otherPin, int on_ms, int off_ms){
  unsigned long t0 = millis();
  while (millis()-t0 < PHASE_TIMEOUT){
    if (limitTripped(wantPin)) { coastEL(); return true; }
    if (limitTripped(otherPin)){ coastEL(); Serial.println(F("[HOMING] EL wrong limit HIGH.")); return false; }

    driveEL(toward);
    delay(on_ms);
    coastEL();
    delay(off_ms);

    if (limitTripped(wantPin)) { coastEL(); return true; }
  }
  coastEL();
  Serial.println(F("[HOMING] EL timeout."));
  return false;
}

// AZ -> 0° (AZ_MIN)
bool homeAZ_toMin(){
  Serial.println(F("[HOME] AZ -> MIN (0°)"));
  if (!pulseToLimit_AZ(REV, AZ_MIN, AZ_MAX, FAST_ON_MS, FAST_OFF_MS)) return false;

  // Back off to re-close NC contact
  Serial.println(F("[HOME] AZ back-off..."));
  driveAZ(FWD); delay(BACKOFF_MS); coastAZ();

  // Creep seat
  Serial.println(F("[HOME] AZ creep seat..."));
  if (!pulseToLimit_AZ(REV, AZ_MIN, AZ_MAX, CREEP_ON_MS, CREEP_OFF_MS)) return false;

  Serial.println(F("[HOME] AZ homed."));
  return true;
}

// EL -> 90° (EL_MAX)
bool homeEL_toMax(){
  Serial.println(F("[HOME] EL -> MAX (90°)"));
  if (!pulseToLimit_EL(FWD, EL_MAX, EL_MIN, FAST_ON_MS, FAST_OFF_MS)) return false;

  Serial.println(F("[HOME] EL back-off..."));
  driveEL(REV); delay(BACKOFF_MS); coastEL();

  Serial.println(F("[HOME] EL creep seat..."));
  if (!pulseToLimit_EL(FWD, EL_MAX, EL_MIN, CREEP_ON_MS, CREEP_OFF_MS)) return false;

  Serial.println(F("[HOME] EL homed."));
  return true;
}

//////////////////// Manual overrides (digital) ////////////////////
struct AxisCmd { Dir az; Dir el; };
const unsigned long OV_DB_MS = 5;

bool debouncedHigh(uint8_t pin){
  unsigned long t0 = millis();
  if (digitalRead(pin)==HIGH){
    while (millis()-t0 < OV_DB_MS){
      if (digitalRead(pin)==LOW) return false;
    }
    return true;
  }
  return false;
}

AxisCmd readOverrides(){
  AxisCmd cmd{STOP, STOP};
  bool up    = debouncedHigh(PIN_UP);
  bool down  = debouncedHigh(PIN_DOWN);
  bool left  = debouncedHigh(PIN_LEFT);
  bool right = debouncedHigh(PIN_RIGHT);

  // Elevation
  if (up && !down)        cmd.el = FWD;
  else if (down && !up)   cmd.el = REV;
  else                    cmd.el = STOP;

  // Azimuth
  if (right && !left)     cmd.az = FWD;
  else if (left && !right)cmd.az = REV;
  else                    cmd.az = STOP;

  // Debug every 500ms
  static unsigned long t=0;
  if (millis()-t > 500){
    t = millis();
    Serial.print(F("[OVR] U=")); Serial.print(up);
    Serial.print(F(" D=")); Serial.print(down);
    Serial.print(F(" L=")); Serial.print(left);
    Serial.print(F(" R=")); Serial.println(right);
  }
  return cmd;
}

//////////////////// Setup / Loop ////////////////////
void setup(){
  // Outputs to L298N
  pinMode(AZ_IN1,OUTPUT); pinMode(AZ_IN2,OUTPUT);
  pinMode(EL_IN1,OUTPUT); pinMode(EL_IN2,OUTPUT);

  // Limit sense inputs (ACTIVE-HIGH)
  pinMode(AZ_MIN,INPUT); pinMode(AZ_MAX,INPUT);
  pinMode(EL_MIN,INPUT); pinMode(EL_MAX,INPUT);

  // Manual overrides as digital inputs (requires external 10k pulldowns)
  pinMode(PIN_UP,INPUT);
  pinMode(PIN_DOWN,INPUT);
  pinMode(PIN_LEFT,INPUT);
  pinMode(PIN_RIGHT,INPUT);

  stopAll();
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nTracker — NC interlocks + active-high sensing"));
  Serial.print(F("Boot limits: AZ_MIN=")); Serial.print(digitalRead(AZ_MIN));
  Serial.print(F(" AZ_MAX=")); Serial.print(digitalRead(AZ_MAX));
  Serial.print(F(" EL_MIN=")); Serial.print(digitalRead(EL_MIN));
  Serial.print(F(" EL_MAX=")); Serial.println(digitalRead(EL_MAX));

  bool okEL = homeEL_toMax();   // go vertical first
  bool okAZ = homeAZ_toMin();   // then azimuth home
  Serial.print(F("[HOME] EL=")); Serial.print(okEL?F("OK"):F("FAIL"));
  Serial.print(F("  AZ="));     Serial.println(okAZ?F("OK"):F("FAIL"));
}

void loop(){
  AxisCmd ac = readOverrides();
  driveAZ(ac.az);
  driveEL(ac.el);
  if (ac.az==STOP && ac.el==STOP) stopAll();
  delay(8);
}
