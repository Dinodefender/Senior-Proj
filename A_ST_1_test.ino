// --- Fix for Arduino auto-prototypes (predeclare the struct) ---
struct AxisCmd;

/*
  Dual-Axis Solar Tracker — SAFE homing + digital overrides + FILTERED LIMITS
  ---------------------------------------------------------------------------
  Wiring (your current setup):
    - Elevation motor on OUT1/OUT2, red→OUT1, black→OUT2  (normal polarity)
    - Azimuth   motor on OUT4/OUT3, red→OUT4, black→OUT3 (reversed polarity, handled)
  L298N:
    - ENA/ENB jumpers ON (full enable)
    - Limits ACTIVE-HIGH (LOW=free, HIGH=limit) — use 10k pulldowns to GND on each input
  Overrides (with 10k pulldowns to GND):
    - A0=UP, A1=DOWN, A2=LEFT, A3=RIGHT (3.3V HIGH)
  Homing: AZ -> 0° (AZ_MIN), EL -> 90° (EL_MAX)
  Safety: pulse-step, fast-brake, press-in validation, watchdogs
  NEW:   filtered limit reads to prevent false early stops (+ debug messages)
*/

// ----- Pin map (matches your wiring) -----
#define AZ_IN1 4   // goes to L298N IN3  (AZ channel, reversed mapping handled)
#define AZ_IN2 3   // goes to L298N IN4
#define EL_IN1 8   // goes to L298N IN1  (EL channel, normal mapping)
#define EL_IN2 7   // goes to L298N IN2

// Limits (ACTIVE-HIGH)
#define AZ_MIN 9
#define AZ_MAX 10
#define EL_MIN 11
#define EL_MAX 12

// Digital override inputs (require external 10k pulldowns)
#define PIN_UP     A0
#define PIN_DOWN   A1
#define PIN_LEFT   A2
#define PIN_RIGHT  A3

// Detection & timing
enum Dir { STOP=0, FWD=1, REV=-1 }; // FWD = AZ CW / EL UP; REV = AZ CCW / EL DOWN
const unsigned long PHASE_TIMEOUT = 12000; // ms per homing phase

// Pulse timing (homing)
const uint8_t  FAST_ON_MS   = 60;
const uint8_t  FAST_OFF_MS  = 10;
const uint8_t  CREEP_ON_MS  = 10;
const uint8_t  CREEP_OFF_MS = 20;
const uint16_t BRAKE_MS     = 150;
const uint16_t BACKOFF_MS   = 200;

// Limit “pressed-in” validation
const uint16_t LIMIT_HOLD_MS     = 80;
const uint8_t  SEAT_EXTRA_PULSES = 4;

// Filter for limit inputs (prevents false guards)
const uint16_t LIMIT_FILTER_MS   = 30;  // must stay HIGH this long to count

// ---- helpers ----
inline bool rawHigh(int p){ return digitalRead(p)==HIGH; }

bool heldHigh_ms(int pin, uint16_t ms){
  unsigned long t0 = millis();
  while (millis()-t0 < ms){
    if (digitalRead(pin)==LOW) return false;
    delay(2);
  }
  return true;
}

// filtered/trusted view of a limit input
bool limitTripped(int pin){
  if (digitalRead(pin)==HIGH) return heldHigh_ms(pin, LIMIT_FILTER_MS);
  return false;
}

void coastAZ(){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,LOW); }
void coastEL(){ digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,LOW); }
void brakeAZ(){ digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,HIGH); delay(BRAKE_MS); coastAZ(); }
void brakeEL(){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,HIGH); delay(BRAKE_MS); coastEL(); }

// Guarded drive using FILTERED limits; logs why it blocks
void driveAZ_guarded(Dir d){
  bool minHit = limitTripped(AZ_MIN);
  bool maxHit = limitTripped(AZ_MAX);

  if (d==REV && minHit){ coastAZ(); static uint32_t t=0; if(millis()-t>300){ t=millis(); Serial.println(F("[GUARD] AZ blocked: AZ_MIN=HIGH")); } return; }
  if (d==FWD && maxHit){ coastAZ(); static uint32_t t=0; if(millis()-t>300){ t=millis(); Serial.println(F("[GUARD] AZ blocked: AZ_MAX=HIGH")); } return; }

  if (d==FWD){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); }  // CW / RIGHT (reversed wiring)
  else if(d==REV){ digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,LOW); } // CCW / LEFT
  else coastAZ();
}

void driveEL_guarded(Dir d){
  bool minHit = limitTripped(EL_MIN);
  bool maxHit = limitTripped(EL_MAX);

  if (d==REV && minHit){ coastEL(); static uint32_t t=0; if(millis()-t>300){ t=millis(); Serial.println(F("[GUARD] EL blocked: EL_MIN=HIGH")); } return; }
  if (d==FWD && maxHit){ coastEL(); static uint32_t t=0; if(millis()-t>300){ t=millis(); Serial.println(F("[GUARD] EL blocked: EL_MAX=HIGH")); } return; }

  if (d==FWD){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,LOW); }   // UP
  else if(d==REV){ digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); } // DOWN
  else coastEL();
}

void stopAll(){ coastAZ(); coastEL(); }

void reportLimitEdge(const char* tag, int pin, int last){
  int now = digitalRead(pin);
  if (now!=last){
    Serial.print(tag); Serial.print(" ");
    Serial.print(pin==AZ_MIN?"AZ_MIN":pin==AZ_MAX?"AZ_MAX":pin==EL_MIN?"EL_MIN":"EL_MAX");
    Serial.print(" -> "); Serial.println(now);
  }
}

// ------------- Pulse-step helpers with press-in validation + filtered checks -------------
bool pulseTowardLimit_AZ(Dir toward, int on_ms, int off_ms, int limitWant, int limitOther){
  unsigned long t0 = millis();
  int lastWant  = digitalRead(limitWant);
  int lastOther = digitalRead(limitOther);

  while (millis()-t0 < PHASE_TIMEOUT){
    // Interlocks (filtered)
    if (limitTripped(limitWant)){ brakeAZ(); return true; }
    if (limitTripped(limitOther)){ brakeAZ(); Serial.println(F("[HOMING] AZ wrong limit already HIGH.")); return false; }

    // ON pulse (reversed mapping)
    if (toward==FWD){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); } // CW/right
    else             { digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,LOW); } // CCW/left
    delay(on_ms);

    int w = digitalRead(limitWant);
    int o = digitalRead(limitOther);
    if (w!=lastWant){  reportLimitEdge("[EDGE]", limitWant, lastWant);   lastWant=w; }
    if (o!=lastOther){ reportLimitEdge("[EDGE]", limitOther, lastOther); lastOther=o; }

    if (limitTripped(limitOther)){ brakeAZ(); Serial.println(F("[HOMING] AZ wrong limit tripped.")); return false; }
    if (limitTripped(limitWant)){
      if (heldHigh_ms(limitWant, LIMIT_HOLD_MS)){
        for (uint8_t i=0;i<SEAT_EXTRA_PULSES;i++){
          if (toward==FWD){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); }
          else             { digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,LOW); }
          delay(6); coastAZ(); delay(12);
          if (!limitTripped(limitWant)) break;
        }
        brakeAZ(); return true;
      }
    }

    // OFF
    coastAZ(); delay(off_ms);

    if (limitTripped(limitOther)){ brakeAZ(); Serial.println(F("[HOMING] AZ wrong limit during OFF.")); return false; }
    if (limitTripped(limitWant)){
      if (heldHigh_ms(limitWant, LIMIT_HOLD_MS)){
        for (uint8_t i=0;i<SEAT_EXTRA_PULSES;i++){
          if (toward==FWD){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); }
          else             { digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,LOW); }
          delay(6); coastAZ(); delay(12);
          if (!limitTripped(limitWant)) break;
        }
        brakeAZ(); return true;
      }
    }
  }
  coastAZ(); Serial.println(F("[HOMING] AZ phase timeout.")); return false;
}

bool pulseTowardLimit_EL(Dir toward, int on_ms, int off_ms, int limitWant, int limitOther){
  unsigned long t0 = millis();
  int lastWant  = digitalRead(limitWant);
  int lastOther = digitalRead(limitOther);

  while (millis()-t0 < PHASE_TIMEOUT){
    if (limitTripped(limitWant)){ brakeEL(); return true; }
    if (limitTripped(limitOther)){ brakeEL(); Serial.println(F("[HOMING] EL wrong limit already HIGH.")); return false; }

    // ON pulse (normal mapping)
    if (toward==FWD){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,LOW); }  // UP
    else             { digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); } // DOWN
    delay(on_ms);

    int w = digitalRead(limitWant);
    int o = digitalRead(limitOther);
    if (w!=lastWant){  reportLimitEdge("[EDGE]", limitWant, lastWant);   lastWant=w; }
    if (o!=lastOther){ reportLimitEdge("[EDGE]", limitOther, lastOther); lastOther=o; }

    if (limitTripped(limitOther)){ brakeEL(); Serial.println(F("[HOMING] EL wrong limit tripped.")); return false; }
    if (limitTripped(limitWant)){
      if (heldHigh_ms(limitWant, LIMIT_HOLD_MS)){
        for (uint8_t i=0;i<SEAT_EXTRA_PULSES;i++){
          if (toward==FWD){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,LOW); }
          else             { digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); }
          delay(6); coastEL(); delay(12);
          if (!limitTripped(limitWant)) break;
        }
        brakeEL(); return true;
      }
    }

    coastEL(); delay(off_ms);

    if (limitTripped(limitOther)){ brakeEL(); Serial.println(F("[HOMING] EL wrong limit during OFF.")); return false; }
    if (limitTripped(limitWant)){
      if (heldHigh_ms(limitWant, LIMIT_HOLD_MS)){
        for (uint8_t i=0;i<SEAT_EXTRA_PULSES;i++){
          if (toward==FWD){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,LOW); }
          else             { digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); }
          delay(6); coastEL(); delay(12);
          if (!limitTripped(limitWant)) break;
        }
        brakeEL(); return true;
      }
    }
  }
  coastEL(); Serial.println(F("[HOMING] EL phase timeout.")); return false;
}

// ------------- High-level homing sequences -------------
bool homeAZ_toMin(){
  Serial.println(F("[HOME] AZ -> MIN (0°): fast approach..."));
  if (!pulseTowardLimit_AZ(REV, FAST_ON_MS, FAST_OFF_MS, AZ_MIN, AZ_MAX)) return false;

  Serial.println(F("[HOME] AZ back-off..."));
  // Back off away from MIN: that's FWD on AZ (with reversed mapping)
  digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); delay(BACKOFF_MS); coastAZ();

  Serial.println(F("[HOME] AZ creep seat..."));
  if (!pulseTowardLimit_AZ(REV, CREEP_ON_MS, CREEP_OFF_MS, AZ_MIN, AZ_MAX)) return false;

  Serial.println(F("[HOME] AZ homed."));
  return true;
}

bool homeEL_toMax(){
  Serial.println(F("[HOME] EL -> MAX (90°): fast approach..."));
  if (!pulseTowardLimit_EL(FWD, FAST_ON_MS, FAST_OFF_MS, EL_MAX, EL_MIN)) return false;

  Serial.println(F("[HOME] EL back-off..."));
  // Back off away from MAX: that's REV on EL (normal mapping)
  digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); delay(BACKOFF_MS); coastEL();

  Serial.println(F("[HOME] EL creep seat..."));
  if (!pulseTowardLimit_EL(FWD, CREEP_ON_MS, CREEP_OFF_MS, EL_MAX, EL_MIN)) return false;

  Serial.println(F("[HOME] EL homed."));
  return true;
}

// ------------- Digital overrides (with debounce) -------------
struct AxisCmd { Dir az; Dir el; };
const unsigned long OV_DB_MS = 5;

bool debouncedHigh(uint8_t pin){
  unsigned long t0 = millis();
  if (digitalRead(pin) == HIGH){
    while (millis() - t0 < OV_DB_MS){
      if (digitalRead(pin) == LOW) return false;
    }
    return true;
  }
  return false;
}

AxisCmd readOverridesDigital(){
  AxisCmd cmd{STOP, STOP};

  bool up    = debouncedHigh(PIN_UP);
  bool down  = debouncedHigh(PIN_DOWN);
  bool left  = debouncedHigh(PIN_LEFT);
  bool right = debouncedHigh(PIN_RIGHT);

  // Elevation
  if (up && !down)        cmd.el = FWD;   // UP
  else if (down && !up)   cmd.el = REV;   // DOWN
  else                    cmd.el = STOP;

  // Azimuth
  if (right && !left)     cmd.az = FWD;   // RIGHT/CW
  else if (left && !right)cmd.az = REV;   // LEFT/CCW
  else                    cmd.az = STOP;

  // Debug every 500 ms
  static unsigned long tDbg=0;
  if (millis()-tDbg > 500){
    tDbg = millis();
    Serial.print(F("[OVR] U="));  Serial.print(up);
    Serial.print(F(" D="));       Serial.print(down);
    Serial.print(F(" L="));       Serial.print(left);
    Serial.print(F(" R="));       Serial.println(right);
  }
  return cmd;
}

// ------------- Setup / Loop -------------
void setup(){
  // Outputs
  pinMode(AZ_IN1,OUTPUT); pinMode(AZ_IN2,OUTPUT);
  pinMode(EL_IN1,OUTPUT); pinMode(EL_IN2,OUTPUT);

  // Limits
  pinMode(AZ_MIN,INPUT); pinMode(AZ_MAX,INPUT);
  pinMode(EL_MIN,INPUT); pinMode(EL_MAX,INPUT);

  // Digital overrides (expect external pulldowns)
  pinMode(PIN_UP,INPUT);
  pinMode(PIN_DOWN,INPUT);
  pinMode(PIN_LEFT,INPUT);
  pinMode(PIN_RIGHT,INPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nTracker SAFE homing + digital overrides + FILTERED LIMITS"));
  Serial.println(F("Limits ACTIVE-HIGH (LOW=free, HIGH=limit). ENA/ENB jumpers ON."));
  Serial.print(F("Boot limits (raw): AZ_MIN=")); Serial.print(digitalRead(AZ_MIN));
  Serial.print(F(" AZ_MAX=")); Serial.print(digitalRead(AZ_MAX));
  Serial.print(F(" EL_MIN=")); Serial.print(digitalRead(EL_MIN));
  Serial.print(F(" EL_MAX=")); Serial.println(digitalRead(EL_MAX));

  bool okAZ = homeAZ_toMin();
  bool okEL = homeEL_toMax();
  Serial.print(F("[HOME] AZ=")); Serial.print(okAZ?F("OK"):F("FAIL"));
  Serial.print(F("  EL=")); Serial.println(okEL?F("OK"):F("FAIL"));
  Serial.println(F("Overrides ready: A0=UP  A1=DOWN  A2=LEFT  A3=RIGHT (3.3V HIGH)"));
}

void loop(){
  AxisCmd ac = readOverridesDigital();
  driveAZ_guarded(ac.az);
  driveEL_guarded(ac.el);
  if (ac.az==STOP && ac.el==STOP) stopAll();
  delay(8);
}
