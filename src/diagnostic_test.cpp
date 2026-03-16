#include "config.h"
#include <Arduino.h>


// ============================================================
//  DIAGNOSTIC TEST FIRMWARE
//  Upload this to test each component individually.
//  Open Serial Monitor at 115200 baud and follow prompts.
// ============================================================

// ── Test state machine ──
enum TestPhase {
  TEST_WELCOME,
  TEST_LEFT_MOTOR_FWD,
  TEST_LEFT_MOTOR_BWD,
  TEST_RIGHT_MOTOR_FWD,
  TEST_RIGHT_MOTOR_BWD,
  TEST_BOTH_FWD,
  TEST_SENSORS_LIVE,
  TEST_ENCODERS_LIVE,
  TEST_DONE
};

TestPhase phase = TEST_WELCOME;
unsigned long phaseStart = 0;
volatile long encL = 0, encR = 0;

void IRAM_ATTR encISR_L() { encL++; }
void IRAM_ATTR encISR_R() { encR++; }

// ── Motor helpers (direct, no classes) ──
void motorLeft(int pwm) {
  if (pwm >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    pwm = -pwm;
  }
  ledcWrite(PWM_CH_L, constrain(pwm, 0, 255));
}

void motorRight(int pwm) {
  if (pwm >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    pwm = -pwm;
  }
  ledcWrite(PWM_CH_R, constrain(pwm, 0, 255));
}

void stopAll() {
  motorLeft(0);
  motorRight(0);
}

// ── Ultrasonic read ──
float readUS() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long d = pulseIn(US_ECHO, HIGH, 30000);
  if (d == 0)
    return -1;
  return (d * 0.0343f) / 2.0f;
}

void nextPhase(TestPhase p) {
  stopAll();
  phase = p;
  phaseStart = millis();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_EN, PWM_CH_L);
  ledcAttachPin(MOTOR_R_EN, PWM_CH_R);

  // Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_REAR_LEFT, INPUT);
  pinMode(IR_REAR_RIGHT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  // Encoders
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_R_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encISR_R, RISING);

  stopAll();

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  TABLE-BOT DIAGNOSTIC TEST");
  Serial.println("==========================================");
  Serial.println();
  Serial.println("Hold the robot in your hand (wheels off the table)");
  Serial.println("for motor tests so it doesn't drive off!");
  Serial.println();
  Serial.println("Send any character in Serial Monitor to start...");
  phase = TEST_WELCOME;
}

void loop() {
  unsigned long elapsed = millis() - phaseStart;

  switch (phase) {

  // ── Wait for user ──
  case TEST_WELCOME:
    if (Serial.available()) {
      while (Serial.available())
        Serial.read(); // flush
      nextPhase(TEST_LEFT_MOTOR_FWD);
    }
    break;

  // ── Left motor forward ──
  case TEST_LEFT_MOTOR_FWD:
    if (elapsed == 0 || (elapsed < 50)) {
      Serial.println(
          "\n--- TEST 1: LEFT motor FORWARD (PWM 180) for 2 sec ---");
      Serial.println("Expected: Left wheel spins FORWARD");
      motorLeft(180);
    }
    if (elapsed > 2000) {
      stopAll();
      Serial.println("Did the LEFT wheel spin FORWARD? (Note the direction)");
      Serial.println();
      nextPhase(TEST_LEFT_MOTOR_BWD);
    }
    break;

  // ── Left motor backward ──
  case TEST_LEFT_MOTOR_BWD:
    if (elapsed < 50) {
      Serial.println("--- TEST 2: LEFT motor BACKWARD (PWM 180) for 2 sec ---");
      Serial.println(
          "Expected: Left wheel spins BACKWARD (opposite of test 1)");
      motorLeft(-180);
    }
    if (elapsed > 2000) {
      stopAll();
      Serial.println("Did the LEFT wheel spin in the OPPOSITE direction?");
      Serial.println();
      delay(500);
      nextPhase(TEST_RIGHT_MOTOR_FWD);
    }
    break;

  // ── Right motor forward ──
  case TEST_RIGHT_MOTOR_FWD:
    if (elapsed < 50) {
      Serial.println("--- TEST 3: RIGHT motor FORWARD (PWM 180) for 2 sec ---");
      Serial.println("Expected: Right wheel spins FORWARD");
      motorRight(180);
    }
    if (elapsed > 2000) {
      stopAll();
      Serial.println("Did the RIGHT wheel spin FORWARD?");
      Serial.println();
      nextPhase(TEST_RIGHT_MOTOR_BWD);
    }
    break;

  // ── Right motor backward ──
  case TEST_RIGHT_MOTOR_BWD:
    if (elapsed < 50) {
      Serial.println(
          "--- TEST 4: RIGHT motor BACKWARD (PWM 180) for 2 sec ---");
      motorRight(-180);
    }
    if (elapsed > 2000) {
      stopAll();
      Serial.println("Did the RIGHT wheel spin BACKWARD?");
      Serial.println();
      delay(500);
      nextPhase(TEST_BOTH_FWD);
    }
    break;

  // ── Both motors forward ──
  case TEST_BOTH_FWD:
    if (elapsed < 50) {
      Serial.println("--- TEST 5: BOTH motors FORWARD (PWM 150) for 2 sec ---");
      Serial.println(
          "Expected: Both wheels spin same direction (robot goes straight)");
      motorLeft(150);
      motorRight(150);
    }
    if (elapsed > 2000) {
      stopAll();
      Serial.println("Did BOTH wheels spin in the SAME direction?");
      Serial.println(
          "If one is reversed, swap its two wires at the L298N terminal.");
      Serial.println();
      delay(500);
      nextPhase(TEST_SENSORS_LIVE);
    }
    break;

  // ── Live sensor reading ──
  case TEST_SENSORS_LIVE:
    if (elapsed < 50) {
      Serial.println("--- TEST 6: LIVE SENSOR READINGS (5 seconds) ---");
      Serial.println("Wave hand in front of ultrasonic sensor.");
      Serial.println("Lift robot to trigger edge sensors.\n");
    }
    if (elapsed < 5000) {
      if (elapsed % 500 < 20) { // Print every ~500ms
        int irL = digitalRead(IR_LEFT);
        int irR = digitalRead(IR_RIGHT);
        float dist = readUS();

        Serial.printf("  IR_LEFT(GPIO%d)=%d  IR_RIGHT(GPIO%d)=%d  |  "
                      "Ultrasonic=%.1f cm\n",
                      IR_LEFT, irL, IR_RIGHT, irR, dist);

        // Interpret edge logic
        bool edgeL = (irL == EDGE_ACTIVE);
        bool edgeR = (irR == EDGE_ACTIVE);
        if (edgeL || edgeR) {
          Serial.printf("  >> EDGE DETECTED: L=%s R=%s\n", edgeL ? "YES" : "no",
                        edgeR ? "YES" : "no");
        }
      }
    }
    if (elapsed >= 5000) {
      Serial.println("\nIMPORTANT: Note the IR values when robot is ON table "
                     "vs lifted off:");
      Serial.println("  - ON table (surface detected): IR should read ___");
      Serial.println("  - OFF table (edge/air):        IR should read ___");
      Serial.printf("  - Currently EDGE_ACTIVE is set to %s\n",
                    EDGE_ACTIVE == HIGH ? "HIGH" : "LOW");
      Serial.println("  - If inverted, change EDGE_ACTIVE in config.h\n");
      delay(1000);
      nextPhase(TEST_ENCODERS_LIVE);
    }
    break;

  // ── Live encoder reading ──
  case TEST_ENCODERS_LIVE:
    if (elapsed < 50) {
      encL = 0;
      encR = 0;
      Serial.println(
          "--- TEST 7: ENCODER TEST (5 sec, both motors at PWM 150) ---");
      Serial.println("Wheels should spin and tick counts increase.\n");
      motorLeft(150);
      motorRight(150);
    }
    if (elapsed < 5000) {
      if (elapsed % 500 < 20) {
        Serial.printf("  Encoder L(GPIO%d)=%ld   Encoder R(GPIO%d)=%ld\n",
                      ENC_L_A, encL, ENC_R_A, encR);
      }
    }
    if (elapsed >= 5000) {
      stopAll();
      Serial.printf("\nFinal ticks: LEFT=%ld  RIGHT=%ld\n", encL, encR);
      if (encL == 0)
        Serial.println("  !! LEFT encoder reads 0 – check wiring/alignment!");
      if (encR == 0)
        Serial.println("  !! RIGHT encoder reads 0 – check wiring/alignment!");
      if (encL > 0 && encR > 0)
        Serial.println("  Both encoders working!");
      Serial.println();
      nextPhase(TEST_DONE);
    }
    break;

  case TEST_DONE:
    if (elapsed < 50) {
      Serial.println("==========================================");
      Serial.println("  ALL TESTS COMPLETE");
      Serial.println("==========================================");
      Serial.println();
      Serial.println("CHECKLIST:");
      Serial.println("  [ ] Both motors spin and reverse correctly");
      Serial.println("  [ ] Both spin SAME direction for forward");
      Serial.println("  [ ] IR sensors change between table/air");
      Serial.println("  [ ] EDGE_ACTIVE polarity is correct");
      Serial.println("  [ ] Ultrasonic distance changes with hand");
      Serial.println("  [ ] Both encoders count ticks > 0");
      Serial.println();
      Serial.println("Copy the serial output and share it so");
      Serial.println("we can fix any issues in the main firmware.");
    }
    break;
  }
}
