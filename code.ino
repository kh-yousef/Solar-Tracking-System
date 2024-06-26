#include <Servo.h>

// Servo definitions
Servo servo_horizontal;
Servo servo_vertical;
Servo servo_vertical_2;

// Initial servo positions
int servoh = 90;
int servov = 90;
int servov_2 = 90;

// LDR pins
#define LTL_PIN A0
#define LTR_PIN A4
#define LBL_PIN A1
#define LBR_PIN A3

// Servo movement limits
#define SERVO_H_LIMIT_H 140
#define SERVO_H_LIMIT_L 50
#define SERVO_V_LIMIT_H 140
#define SERVO_V_LIMIT_L 50

// Define adjustment sensitivity
#define ADJUST_SENSITIVITY 20  // Increased to reduce minor movements

// Mode timing
unsigned long trackingTime = 60000;     // 1 minute for tracking
unsigned long staticTime = 60000;       // 1 minute for static mode

// Tracking control
unsigned long lastModeSwitch = 0;
unsigned long lastReadingTime = 0;
bool isTrackingMode = true;  // Start in tracking mode

// Static angle for stationary mode
int staticServoH = 90;   // Static horizontal angle
int staticServoV = 145;  // Static vertical angle

// PID constants
double Kp = 0.6, Ki = 0.07, Kd = 0.03;  // Reduced PID gains
double previousErrorH = 0, integralH = 0;
double previousErrorV = 0, integralV = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LTL_PIN, INPUT);
  pinMode(LTR_PIN, INPUT);
  pinMode(LBL_PIN, INPUT);
  pinMode(LBR_PIN, INPUT);
  servo_vertical.attach(7);     // Attach the vertical servo to pin 7
  servo_vertical_2.attach(9);   // Attach the second vertical servo to pin 9
  servo_horizontal.attach(11);  // Attach the horizontal servo to pin 11

  // Move servos to their initial positions
  servoh = 90;
  servov = 90;
  servov_2 = 180 - servov;
  servo_horizontal.write(servoh);
  servo_vertical.write(servov);
  servo_vertical_2.write(servov_2);

  // Record the start time
  lastModeSwitch = millis();

  Serial.println("Timestamp,Mode");
}

void loop() {
  unsigned long currentTime = millis();

  // Check if it's time to switch modes
  if (isTrackingMode && currentTime - lastModeSwitch >= trackingTime) {
    // Switch to static mode
    isTrackingMode = false;
    lastModeSwitch = currentTime;
    // Set servos to the static position
    servoh = staticServoH;  // Ensure horizontal servo stays at static angle
    servo_vertical.write(staticServoV);
    servo_vertical_2.write(180 - staticServoV);
    servo_horizontal.write(servoh);
    // Print mode switch info
    Serial.println("Switching to Static Mode");
  } else if (!isTrackingMode && currentTime - lastModeSwitch >= staticTime) {
    // Switch back to tracking mode
    isTrackingMode = true;
    lastModeSwitch = currentTime;
    // Print mode switch info
    Serial.println("Switching to Tracking Mode");
  }

  if (isTrackingMode) {
    // Tracking Mode: Adjust servos based on LDR values
    adjustServosPID();
    Serial.print("Tracking\n");
  } else {
    // Static Mode: Do nothing, servos are already set
    Serial.print("Stationary\n");
  }

  delay(100);  // Short delay to slow down the adjustments and allow time for reading the monitor
}

void adjustServosPID() {
  int TL_value = analogRead(LTL_PIN);
  int TR_value = analogRead(LTR_PIN);
  int BL_value = analogRead(LBL_PIN);
  int BR_value = analogRead(LBR_PIN);

  // Calculate averages
  int T_value = (TL_value + TR_value) / 2;
  int B_value = (BL_value + BR_value) / 2;
  int L_value = (TL_value + BL_value) / 2;
  int R_value = (TR_value + BR_value) / 2;

  // Calculate errors
  int errorH = L_value - R_value;
  int errorV = T_value - B_value;

  // PID calculations for horizontal adjustment
  integralH += errorH;
  double derivativeH = errorH - previousErrorH;
  double adjustmentH = Kp * errorH + Ki * integralH + Kd * derivativeH;
  previousErrorH = errorH;

  // PID calculations for vertical adjustment
  integralV += errorV;
  double derivativeV = errorV - previousErrorV;
  double adjustmentV = Kp * errorV + Ki * integralV + Kd * derivativeV;
  previousErrorV = errorV;

  // Check if adjustments are necessary based on sensitivity
  if (abs(errorH) > ADJUST_SENSITIVITY) {
    servoh = constrain(servoh + adjustmentH, SERVO_H_LIMIT_L, SERVO_H_LIMIT_H);
  }

  if (abs(errorV) > ADJUST_SENSITIVITY) {
    servov = constrain(servov + adjustmentV, SERVO_V_LIMIT_L, SERVO_V_LIMIT_H);
    servov_2 = 180 - servov;  // Keep servov_2 as the opposite of servov
  }

  // Update servo positions
  servo_horizontal.write(servoh);
  servo_vertical.write(servov);
  servo_vertical_2.write(servov_2);

  // Debugging output
  // Serial.print("TL: "); Serial.print(TL_value);
  // Serial.print("\tTR: "); Serial.print(TR_value);
  // Serial.print("\tBL: "); Serial.print(BL_value);
  // Serial.print("\tBR: "); Serial.print(BR_value);
  // Serial.print("\tT: "); Serial.print(T_value);
  // Serial.print("\tL: "); Serial.print(L_value);
  // Serial.print("\tB: "); Serial.print(B_value);
  // Serial.print("\tR: "); Serial.print(R_value);
  // Serial.print("\tservov: "); Serial.print(servov);
  // Serial.print("\tservoh: "); Serial.println(servoh);
}
