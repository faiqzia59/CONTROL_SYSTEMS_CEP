/* ============================================================
   INVERTED PENDULUM – SMOOTH LQR + CART CENTERING + LIMIT PROTECTION
   Arduino Mega 2560

   PHYSICAL SETUP:
   - Track length     : 1250 mm
   - Pulley diameter  : 40 mm  → circumference = 125.66 mm/rev
   - Encoder PPR      : 2048 counts/rev
   - Counts per mm    : 2048 / 125.66 = ~16.3 counts/mm
   - Total counts     : 1250 × 16.3 = ~20,370 counts (full track)
   - Cart CENTER      : ~10,185 counts  (50% of track)
   - Soft limit       : ~16,296 counts  (80% of track) — start braking
   - Hard limit       : ~18,333 counts  (90% of track) — force reverse
   - START POSITION   : Cart placed at LEFT END → cartCounts = 0

   NOTE: Cart starts at LEFT END. Controller will immediately begin
         driving cart toward center (10,185) before balancing.
   ============================================================ */

#include <Arduino.h>

float K1 = 0.00015;   // cart position gain (centering)
float K2 = 0.008;     // cart velocity gain
float K3 = 8.0;       // pendulum angle gain (reduced for smoothness)
float K4 = 0.6;       // angular velocity gain (reduced)

// PWM limits
const int MAX_PWM = 80;
const float PWM_SCALE = 0.7;  // Soft scaling to reduce jerks

const int BALANCE_WINDOW = 80;   // Only try to balance within ±80 counts
const int UPRIGHT_DEADZONE = 2;  // Ignore tiny deviations to stop micro-corrections


// ===================== TRACK / LIMIT SETTINGS =====================
// All values in encoder counts. Cart starts at LEFT END (count = 0).
const long  TRACK_TOTAL     = 20370;  // Full track in counts (~1250mm)
const long  TRACK_CENTER    = 10185;  // Target center position
const long  CART_SOFT_LIMIT_LOW  = 2000;   // Left soft limit  (~123mm from left end)
const long  CART_SOFT_LIMIT_HIGH = 18296;  // Right soft limit (~80% of track)
const long  CART_HARD_LIMIT_LOW  = 1000;   // Left hard limit  (~61mm from left end)
const long  CART_HARD_LIMIT_HIGH = 19333;  // Right hard limit (~90% of track)
const float LIMIT_PENALTY   = 4.0;         // Extra centering gain multiplier near boundary

// ===================== PINS =====================
const int cartEncA = 2;
const int cartEncB = 3;
const int pendEncA = 18;
const int pendEncB = 19;

const int R_EN  = 7;
const int L_EN  = 8;
const int R_PWM = 9;
const int L_PWM = 10;

// ===================== VARIABLES =====================
volatile long pendCounts = 0;
volatile long cartCounts = 0;

long lastPend = 0;
long lastCart = 0;

float theta_dot_filtered = 0.0;
float x_dot_filtered     = 0.0;

unsigned long lastLoopTime = 0;

// Low-pass filter coefficient (0 < alpha < 1)
const float alpha = 0.85;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // Motor driver enable
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
  pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);

  // Encoder pins
  pinMode(cartEncA, INPUT); pinMode(cartEncB, INPUT);
  pinMode(pendEncA, INPUT); pinMode(pendEncB, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(cartEncA), ISR_cart,     RISING);
  attachInterrupt(digitalPinToInterrupt(pendEncA), ISR_pendulum, RISING);

  // Zero both encoders
  // Cart starts at LEFT END of track → count = 0
  // Pendulum should be held upright manually before release
  noInterrupts();
  pendCounts = 0;
  cartCounts = 0;
  interrupts();

  lastLoopTime = millis();
  lastPend = 0;
  lastCart = 0;

  Serial.println("============================================");
  Serial.println(" Inverted Pendulum – Final Code");
  Serial.println(" Cart start: LEFT END (count=0)");
  Serial.println(" Cart center target: 10185 counts");
  Serial.println(" Soft limits: 2000 | 18296");
  Serial.println(" Hard limits: 1000 | 19333");
  Serial.println(" Raise pendulum to upright now...");
  Serial.println(" Balancing begins in 5 seconds.");
  Serial.println("============================================");
  delay(5000);
}

// ===================== MAIN LOOP =====================
void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < 2) return;  // ~500 Hz loop
  float dt = (now - lastLoopTime) / 1000.0;
  lastLoopTime = now;

  // ===== Read encoders safely =====
  noInterrupts();
  long currentPend = pendCounts;
  long currentCart = cartCounts;
  interrupts();

  // ===== Normalize pendulum angle =====
  long normalizedPend = currentPend % 2048;
  if (normalizedPend < 0) normalizedPend += 2048;

  // ===== Compute raw velocities =====
  float x_dot_raw     = (currentCart - lastCart) / dt;
  float theta_dot_raw = -(normalizedPend - lastPend) / dt;  // upright = positive

  // ===== Low-pass filter velocities =====
  x_dot_filtered     = alpha * x_dot_filtered     + (1 - alpha) * x_dot_raw;
  theta_dot_filtered = alpha * theta_dot_filtered + (1 - alpha) * theta_dot_raw;

  lastPend = normalizedPend;
  lastCart = currentCart;

  // ===== State variables =====
  // x is cart error from CENTER (so controller always tries to return to center)
  float x     = currentCart - TRACK_CENTER;
  float theta = 1024.0 - normalizedPend;  // 0 = upright

  // ===== Serial debug (comment out to reduce overhead) =====
  // Serial.print("Cart:"); Serial.print(currentCart);
  // Serial.print(" Theta:"); Serial.print(theta);
  // Serial.print(" x_err:"); Serial.println(x);

  // ===== HARD LIMIT: Emergency override regardless of pendulum state =====
  if (currentCart <= CART_HARD_LIMIT_LOW) {
    // Too close to LEFT end — force RIGHT immediately
    driveMotor(30);
    return;
  }
  if (currentCart >= CART_HARD_LIMIT_HIGH) {
    // Too close to RIGHT end — force LEFT immediately
    driveMotor(-30);
    return;
  }

  // ===== BALANCE WINDOW: Only attempt balance if pendulum is near upright =====
  if (abs(theta) < BALANCE_WINDOW) {

    // === Adaptive centering gain based on cart proximity to limits ===
    float dynamicK1 = K1;

    if (currentCart < CART_SOFT_LIMIT_LOW) {
      // Approaching left soft limit — boost centering to pull RIGHT
      float excess = (float)(CART_SOFT_LIMIT_LOW - currentCart);
      float range  = (float)(CART_SOFT_LIMIT_LOW - CART_HARD_LIMIT_LOW);
      dynamicK1 = K1 + LIMIT_PENALTY * K1 * (excess / range);
    }
    else if (currentCart > CART_SOFT_LIMIT_HIGH) {
      // Approaching right soft limit — boost centering to pull LEFT
      float excess = (float)(currentCart - CART_SOFT_LIMIT_HIGH);
      float range  = (float)(CART_HARD_LIMIT_HIGH - CART_SOFT_LIMIT_HIGH);
      dynamicK1 = K1 + LIMIT_PENALTY * K1 * (excess / range);
    }

    // === LQR Control Law ===
    // x is already relative to center, so K1 naturally drives cart back to center
    float u = -(dynamicK1 * x
              + K2 * x_dot_filtered
              + K3 * theta
              + K4 * theta_dot_filtered);

    // === Deadzone at upright to prevent micro-jitter ===
    if (abs(theta) < UPRIGHT_DEADZONE) u = 0;

    driveMotor(u * PWM_SCALE);

  } else {
    // Pendulum has fallen — stop motor
    driveMotor(0);
  }
}

// ===================== MOTOR DRIVER =====================
void driveMotor(float pwm) {
  int output = constrain((int)pwm, -MAX_PWM, MAX_PWM);

  if (output > 0) {
    analogWrite(R_PWM, output);
    analogWrite(L_PWM, 0);
  } else if (output < 0) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, abs(output));
  } else {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
  }
}

// ===================== INTERRUPTS =====================
void ISR_cart() {
  if (digitalRead(cartEncB) == HIGH) cartCounts++;
  else cartCounts--;
}

void ISR_pendulum() {
  if (digitalRead(pendEncB) == HIGH) pendCounts++;
  else pendCounts--;
}
