/*
 * AMR Motor Control - Arduino Firmware
 *
 * Handles:
 * - 2x Cytron MD10C single-channel motor drivers (PWM + DIR each)
 * - Hall effect encoder reading via interrupts
 * - PID velocity control per motor
 * - Serial communication with RPi4 ROS2 node
 *
 * Serial Protocol (115200 baud):
 *   RPi -> Arduino: "M:<left_rpm>,<right_rpm>\n"
 *   Arduino -> RPi: "E:<left_ticks>,<right_ticks>,<dt_ms>\n"
 *
 * Wiring Diagram:
 *
 *   Arduino Uno          MD10C #1 (Left)       Left Motor
 *   ──────────           ──────────────        ──────────
 *   D5 (PWM) ──────────► PWM                   M+ ◄──── OUT+
 *   D4       ──────────► DIR                   M- ◄──── OUT-
 *   D2 (INT0) ◄──────── Encoder A              Enc A ──►
 *   D8        ◄──────── Encoder B              Enc B ──►
 *   GND ────────────────► GND
 *
 *   Arduino Uno          MD10C #2 (Right)      Right Motor
 *   ──────────           ───────────────       ───────────
 *   D6 (PWM) ──────────► PWM                   M+ ◄──── OUT+
 *   D7       ──────────► DIR                   M- ◄──── OUT-
 *   D3 (INT1) ◄──────── Encoder A              Enc A ──►
 *   D9        ◄──────── Encoder B              Enc B ──►
 *   GND ────────────────► GND
 *
 *   Power: Both MD10C drivers powered from battery (7-24V)
 *          Arduino powered via USB from RPi4
 *          MD10C logic GND connected to Arduino GND
 */

// --- Pin Definitions (2x Cytron MD10C) ---
// MD10C #1 = Left motor, MD10C #2 = Right motor
#define LEFT_PWM  5    // MD10C #1 PWM input
#define LEFT_DIR  4    // MD10C #1 DIR input
#define RIGHT_PWM 6    // MD10C #2 PWM input
#define RIGHT_DIR 7    // MD10C #2 DIR input

#define LEFT_ENC_A  2   // INT0
#define LEFT_ENC_B  8
#define RIGHT_ENC_A 3   // INT1
#define RIGHT_ENC_B 9

// --- Motor & Encoder Config ---
// MEASURE THIS: rotate wheel exactly 1 revolution, count ticks
#define TICKS_PER_REV 330

// --- PID Tuning ---
// Start with Kp only, add Ki if needed, Kd usually unnecessary
float Kp = 2.0;
float Ki = 5.0;
float Kd = 0.0;

// --- Timing ---
#define CONTROL_INTERVAL_MS 20   // 50 Hz
#define REPORT_INTERVAL_MS  20   // 50 Hz
#define CMD_TIMEOUT_MS      500  // Safety stop

// --- Global State ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

float target_left_rpm = 0.0;
float target_right_rpm = 0.0;

// PID state
float left_integral = 0.0, left_prev_error = 0.0;
float right_integral = 0.0, right_prev_error = 0.0;
long prev_left_ticks = 0, prev_right_ticks = 0;

unsigned long last_control_time = 0;
unsigned long last_report_time = 0;
unsigned long last_cmd_time = 0;

// Serial input buffer
char serial_buf[64];
int serial_idx = 0;

// --- Interrupt Service Routines ---
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B))
    left_ticks++;
  else
    left_ticks--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B))
    right_ticks++;
  else
    right_ticks--;
}

// --- Motor Control ---
void setMotor(int pwm_pin, int dir_pin, float output) {
  int pwm_val = constrain(abs((int)output), 0, 255);
  digitalWrite(dir_pin, output >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, pwm_val);
}

float computePID(float target_rpm, float measured_rpm,
                 float &integral, float &prev_error, float dt) {
  float error = target_rpm - measured_rpm;
  integral += error * dt;
  // Anti-windup
  integral = constrain(integral, -100.0, 100.0);
  float derivative = (dt > 0) ? (error - prev_error) / dt : 0.0;
  prev_error = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  return constrain(output, -255.0, 255.0);
}

// --- Serial Parsing ---
void processSerialCommand(char *cmd) {
  if (cmd[0] == 'M' && cmd[1] == ':') {
    // Parse "M:<left_rpm>,<right_rpm>"
    char *ptr = cmd + 2;
    char *comma = strchr(ptr, ',');
    if (comma != NULL) {
      *comma = '\0';
      target_left_rpm = atof(ptr);
      target_right_rpm = atof(comma + 1);
      last_cmd_time = millis();
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_idx > 0) {
        serial_buf[serial_idx] = '\0';
        processSerialCommand(serial_buf);
        serial_idx = 0;
      }
    } else if (serial_idx < (int)sizeof(serial_buf) - 1) {
      serial_buf[serial_idx++] = c;
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Stop motors
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  last_control_time = millis();
  last_report_time = millis();
  last_cmd_time = millis();
}

// --- Main Loop ---
void loop() {
  readSerial();

  unsigned long now = millis();

  // Safety: stop motors if no command received
  if (now - last_cmd_time > CMD_TIMEOUT_MS) {
    target_left_rpm = 0.0;
    target_right_rpm = 0.0;
  }

  // PID control at fixed rate
  if (now - last_control_time >= CONTROL_INTERVAL_MS) {
    float dt = (now - last_control_time) / 1000.0;
    last_control_time = now;

    // Read encoder ticks (atomic copy from volatile)
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();

    // Compute measured RPM
    long dl = lt - prev_left_ticks;
    long dr = rt - prev_right_ticks;
    prev_left_ticks = lt;
    prev_right_ticks = rt;

    float measured_left_rpm = (dl / (float)TICKS_PER_REV) * (60.0 / dt);
    float measured_right_rpm = (dr / (float)TICKS_PER_REV) * (60.0 / dt);

    // PID
    float left_output = computePID(target_left_rpm, measured_left_rpm,
                                    left_integral, left_prev_error, dt);
    float right_output = computePID(target_right_rpm, measured_right_rpm,
                                     right_integral, right_prev_error, dt);

    // If target is zero, force output to zero (don't fight friction)
    if (target_left_rpm == 0.0) {
      left_output = 0.0;
      left_integral = 0.0;
    }
    if (target_right_rpm == 0.0) {
      right_output = 0.0;
      right_integral = 0.0;
    }

    setMotor(LEFT_PWM, LEFT_DIR, left_output);
    setMotor(RIGHT_PWM, RIGHT_DIR, right_output);
  }

  // Report encoder data at fixed rate
  if (now - last_report_time >= REPORT_INTERVAL_MS) {
    unsigned long dt_ms = now - last_report_time;
    last_report_time = now;

    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();

    Serial.print("E:");
    Serial.print(lt);
    Serial.print(",");
    Serial.print(rt);
    Serial.print(",");
    Serial.println(dt_ms);
  }
}
