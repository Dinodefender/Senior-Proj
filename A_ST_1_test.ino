// --- Forward declare to avoid Arduino auto-prototype issues ---
struct AxisCmd;

/*
  Dual-Axis Solar Tracker — NC Interlocks + Photodiode Tracking @9600 baud
  ------------------------------------------------------------------------
  Wiring:
    AZ motor: RED→OUT4, BLACK→OUT3  (D3→IN4 = CW / Right, D4→IN3 = CCW / Left)
    EL motor: RED→OUT1, BLACK→OUT2  (D8→IN1 = Up, D7→IN2 = Down)
    Limits (ACTIVE-HIGH): AZ_MIN = D9, AZ_MAX = D10, EL_MIN = D11, EL_MAX = D12
    Each NC contact physically breaks its direction line.

  Photodiodes (quad), per your mapping:
    UL → A1, UR → A2, LL → A3, LR → A4
*/

// -------------------- PINS --------------------
#define AZ_IN1 4   // L298N IN3 (AZ CCW / Left)
#define AZ_IN2 3   // L298N IN4 (AZ CW  / Right)
#define EL_IN1 8   // L298N IN1 (EL Up)
#define EL_IN2 7   // L298N IN2 (EL Down)

// Limit sensing (ACTIVE-HIGH)
#define AZ_MIN 9
#define AZ_MAX 10
#define EL_MIN 11
#define EL_MAX 12

// Photodiodes (quad)
#define PD_UL A1  // UL
#define PD_UR A2  // UR
#define PD_LL A3  // LL
#define PD_LR A4  // LR

// -------------------- CONSTANTS --------------------
enum Dir { STOP=0, FWD=1, REV=-1 };   // FWD = AZ CW / EL Up; REV = AZ CCW / EL Down

// Homing timing
const uint8_t  FAST_ON_MS   = 50;
const uint8_t  FAST_OFF_MS  = 10;
const uint8_t  CREEP_ON_MS  = 8;
const uint8_t  CREEP_OFF_MS = 20;
const uint16_t BACKOFF_MS   = 180;
const unsigned long PHASE_TIMEOUT = 12000;

// Limit debounce
const uint16_t LIMIT_FILTER_MS = 20;

// Tracking anti-jitter (tuned to reduce hitching)
const float     EMA_ALPHA          = 0.25f;  // lower (e.g., 0.18) for more smoothing
const int       DEAD_BAND          = 40;
const int       HYSTERESIS_MARGIN  = 25;
const int       MAX_ERR_CLAMP      = 300;
const uint8_t   TRACK_ON_MS_MIN    = 8;
const uint8_t   TRACK_ON_MS_MAX    = 45;
const uint8_t   TRACK_OFF_MS       = 45;
const uint16_t  MIN_LUX_SUM        = 240;
const uint8_t   REQ_CONSEC_CYCLES  = 3;
const uint16_t  MOVE_COOLDOWN_MS   = 0;
const int       REVERSAL_MARGIN    = 5;     // extra counts needed to reverse direction

// -------------------- GLOBALS --------------------
float ul_f=0, ur_f=0, ll_f=0, lr_f=0;
uint8_t az_ready_count = 0, el_ready_count = 0;
unsigned long last_az_move_ms = 0, last_el_move_ms = 0;
// NEW: remember last commanded directions for hysteresis
Dir last_dir_az = STOP;
Dir last_dir_el = STOP;

// -------------------- HELPERS --------------------
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

// Motor primitives
void coastAZ(){ digitalWrite(AZ_IN1,LOW); digitalWrite(AZ_IN2,LOW); }
void coastEL(){ digitalWrite(EL_IN1,LOW); digitalWrite(EL_IN2,LOW); }

void driveAZ(Dir d){
  if (d==FWD && limitTripped(AZ_MAX)) { coastAZ(); return; }
  if (d==REV && limitTripped(AZ_MIN)) { coastAZ(); return; }
  if (d==FWD){ digitalWrite(AZ_IN1,LOW);  digitalWrite(AZ_IN2,HIGH); } // CW / Right
  else if(d==REV){ digitalWrite(AZ_IN1,HIGH); digitalWrite(AZ_IN2,LOW); } // CCW / Left
  else coastAZ();
}
void driveEL(Dir d){
  if (d==FWD && limitTripped(EL_MAX)) { coastEL(); return; }
  if (d==REV && limitTripped(EL_MIN)) { coastEL(); return; }
  if (d==FWD){ digitalWrite(EL_IN1,HIGH); digitalWrite(EL_IN2,LOW); }    // Up
  else if(d==REV){ digitalWrite(EL_IN1,LOW);  digitalWrite(EL_IN2,HIGH); } // Down
  else coastEL();
}

// -------------------- HOMING --------------------
bool pulseToLimit_AZ(Dir toward,int wantPin,int otherPin,int on_ms,int off_ms){
  unsigned long t0=millis();
  while(millis()-t0<PHASE_TIMEOUT){
    if(limitTripped(wantPin)){coastAZ();return true;}
    if(limitTripped(otherPin)){coastAZ();Serial.println(F("[HOMING] AZ wrong limit."));return false;}
    driveAZ(toward);delay(on_ms);coastAZ();delay(off_ms);
    if(limitTripped(wantPin)){coastAZ();return true;}
  }
  coastAZ();Serial.println(F("[HOMING] AZ timeout."));return false;
}
bool pulseToLimit_EL(Dir toward,int wantPin,int otherPin,int on_ms,int off_ms){
  unsigned long t0=millis();
  while(millis()-t0<PHASE_TIMEOUT){
    if(limitTripped(wantPin)){coastEL();return true;}
    if(limitTripped(otherPin)){coastEL();Serial.println(F("[HOMING] EL wrong limit."));return false;}
    driveEL(toward);delay(on_ms);coastEL();delay(off_ms);
    if(limitTripped(wantPin)){coastEL();return true;}
  }
  coastEL();Serial.println(F("[HOMING] EL timeout."));return false;
}
bool homeAZ_toMin(){
  Serial.println(F("[HOME] AZ → MIN (0°)"));
  if(!pulseToLimit_AZ(REV,AZ_MIN,AZ_MAX,FAST_ON_MS,FAST_OFF_MS))return false;
  Serial.println(F("[HOME] AZ back-off…"));driveAZ(FWD);delay(BACKOFF_MS);coastAZ();
  Serial.println(F("[HOME] AZ creep seat…"));
  if(!pulseToLimit_AZ(REV,AZ_MIN,AZ_MAX,CREEP_ON_MS,CREEP_OFF_MS))return false;
  Serial.println(F("[HOME] AZ homed."));return true;
}
bool homeEL_toMax(){
  Serial.println(F("[HOME] EL → MAX (90°)"));
  if(!pulseToLimit_EL(FWD,EL_MAX,EL_MIN,FAST_ON_MS,FAST_OFF_MS))return false;
  Serial.println(F("[HOME] EL back-off…"));driveEL(REV);delay(BACKOFF_MS);coastEL();
  Serial.println(F("[HOME] EL creep seat…"));
  if(!pulseToLimit_EL(FWD,EL_MAX,EL_MIN,CREEP_ON_MS,CREEP_OFF_MS))return false;
  Serial.println(F("[HOME] EL homed."));return true;
}

// -------------------- TRACKING --------------------
static uint8_t pulseLenFromErr(int e){
  int mag=abs(e);
  int ms=map(mag,0,MAX_ERR_CLAMP,TRACK_ON_MS_MIN,TRACK_ON_MS_MAX);
  return (uint8_t)constrain(ms,TRACK_ON_MS_MIN,TRACK_ON_MS_MAX);
}

// One anti-jitter, normalized tracking step; returns true if any movement occurred
bool trackStep(){
  // 1) Raw reads
  int pUL = analogRead(PD_UL); // UL
  int pUR = analogRead(PD_UR); // UR
  int pLL = analogRead(PD_LL); // LL
  int pLR = analogRead(PD_LR); // LR

  // 2) EMA smoothing
  if (ul_f==0 && ur_f==0 && ll_f==0 && lr_f==0){ ul_f=pUL; ur_f=pUR; ll_f=pLL; lr_f=pLR; }
  else {
    ul_f = ul_f + EMA_ALPHA * (pUL - ul_f);
    ur_f = ur_f + EMA_ALPHA * (pUR - ur_f);
    ll_f = ll_f + EMA_ALPHA * (pLL - ll_f);
    lr_f = lr_f + EMA_ALPHA * (pLR - lr_f);
  }

  // 3) Sums & total
  float sumL = ul_f + ll_f;
  float sumR = ur_f + lr_f;
  float sumU = ul_f + ur_f;
  float sumD = ll_f + lr_f;
  float total = sumL + sumR; // == sumU + sumD

  // 4) Print (every ~500 ms)
  static unsigned long tPrint=0;
  if (millis()-tPrint > 500){
    tPrint = millis();
    int rawA1 = analogRead(A1);
    int rawA2 = analogRead(A2);
    int rawA3 = analogRead(A3);
    int rawA4 = analogRead(A4);
    Serial.print(F("RAW A1=")); Serial.print(rawA1);
    Serial.print(F(" A2="));    Serial.print(rawA2);
    Serial.print(F(" A3="));    Serial.print(rawA3);
    Serial.print(F(" A4="));    Serial.print(rawA4);
    Serial.print(F("  |  UL=")); Serial.print((int)ul_f);
    Serial.print(F(" UR="));     Serial.print((int)ur_f);
    Serial.print(F(" LL="));     Serial.print((int)ll_f);
    Serial.print(F(" LR="));     Serial.print((int)lr_f);
    Serial.print(F("  | Sum=")); Serial.println((int)total);
  }

  // 5) Low-light hold
  if (total < MIN_LUX_SUM) return false;

  // 6) Pre-normalized errors (for pulse sizing) and normalized errors (for decisions)
  int   errAZ_raw = (int)(sumR - sumL);
  int   errEL_raw = (int)(sumU - sumD);
  float nErrAZ    = (sumR - sumL) / (total + 1.0f); // normalized
  float nErrEL    = (sumU - sumD) / (total + 1.0f);

  // 7) Deadband + hysteresis on the raw error (counts)
  int threshOut = DEAD_BAND + HYSTERESIS_MARGIN;
  bool az_beyond = abs(errAZ_raw) > threshOut;
  bool el_beyond = abs(errEL_raw) > threshOut;

  // Count consecutive beyond-threshold cycles (use Arduino macro min())
  if (az_beyond) az_ready_count = min((uint8_t)255, (uint8_t)(az_ready_count+1));
  else           az_ready_count = 0;
  if (el_beyond) el_ready_count = min((uint8_t)255, (uint8_t)(el_ready_count+1));
  else           el_ready_count = 0;

  // 8) Axis arbitration — move the axis with the larger |normalized error|
  bool az_cool = (millis() - last_az_move_ms) >= MOVE_COOLDOWN_MS;
  bool el_cool = (millis() - last_el_move_ms) >= MOVE_COOLDOWN_MS;

  Dir wantAZ = (nErrAZ > 0) ? FWD : (nErrAZ < 0) ? REV : STOP;
  Dir wantEL = (nErrEL > 0) ? FWD : (nErrEL < 0) ? REV : STOP;

  // Directional hysteresis: require extra margin to reverse direction
  if (last_dir_az != STOP && wantAZ != STOP && wantAZ != last_dir_az) {
    if (abs(errAZ_raw) < (threshOut + REVERSAL_MARGIN)) wantAZ = STOP;
  }
  if (last_dir_el != STOP && wantEL != STOP && wantEL != last_dir_el) {
    if (abs(errEL_raw) < (threshOut + REVERSAL_MARGIN)) wantEL = STOP;
  }

  float magAZ = fabs(nErrAZ);
  float magEL = fabs(nErrEL);
  bool moveAZ = (az_ready_count >= REQ_CONSEC_CYCLES) && az_cool && (magAZ >= magEL) && wantAZ != STOP;
  bool moveEL = (el_ready_count >= REQ_CONSEC_CYCLES) && el_cool && (magEL >  magAZ) && wantEL != STOP;

  bool moved = false;

  // Clamp raw errors for pulse sizing
  errAZ_raw = constrain(errAZ_raw, -MAX_ERR_CLAMP, MAX_ERR_CLAMP);
  errEL_raw = constrain(errEL_raw, -MAX_ERR_CLAMP, MAX_ERR_CLAMP);

  if (moveAZ){
    driveAZ(wantAZ);
    delay(pulseLenFromErr(errAZ_raw));
    coastAZ();
    last_az_move_ms = millis();
    last_dir_az = wantAZ;
    moved = true;
  }

  if (moveEL){
    driveEL(wantEL);
    delay(pulseLenFromErr(errEL_raw));
    coastEL();
    last_el_move_ms = millis();
    last_dir_el = wantEL;
    moved = true;
  }

  delay(moved ? TRACK_OFF_MS : 10);
  return moved;
}

// -------------------- SETUP / LOOP --------------------
void setup(){
  pinMode(AZ_IN1,OUTPUT); pinMode(AZ_IN2,OUTPUT);
  pinMode(EL_IN1,OUTPUT); pinMode(EL_IN2,OUTPUT);

  pinMode(AZ_MIN,INPUT); pinMode(AZ_MAX,INPUT);
  pinMode(EL_MIN,INPUT); pinMode(EL_MAX,INPUT);

  Serial.begin(9600);
  delay(200);
  Serial.println(F("\nTracker: NC interlocks + Photodiode tracking @ 9600 baud"));
  Serial.print(F("Boot limits: AZ_MIN=")); Serial.print(digitalRead(AZ_MIN));
  Serial.print(F(" AZ_MAX="));            Serial.print(digitalRead(AZ_MAX));
  Serial.print(F(" EL_MIN="));            Serial.print(digitalRead(EL_MIN));
  Serial.print(F(" EL_MAX="));            Serial.println(digitalRead(EL_MAX));

  bool okEL = homeEL_toMax();
  bool okAZ = homeAZ_toMin();
  Serial.print(F("[HOME] EL=")); Serial.print(okEL?F("OK"):F("FAIL"));
  Serial.print(F("  AZ="));      Serial.println(okAZ?F("OK"):F("FAIL"));
}

void loop(){
  trackStep();
}
