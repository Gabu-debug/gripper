#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

const uint8_t STEP1 = 9,  DIR1 = 8;
const uint8_t EN_AB = 12;  
// Link for the motor
// https://www.omc-stepperonline.com/nema-17-external-acme-linear-stepper-motor-1-5a-34mm-stack-screw-lead-1-27mm-0-05-lead-length-150mm-17e13s1504ff5-150rs
// ★ The Write-Only Hack: 
// Assign RX to a totally unused pin (e.g., 4) so the library doesn't conflict.
// Pin 5 (TX) goes through your resistor to the PDN_UART pin.
const uint8_t DUMMY_RX_PIN = 4; 
const uint8_t TX_PIN       = 5;

AccelStepper stepper3(AccelStepper::DRIVER, STEP1, DIR1);

#define R_SENSE        0.11f
#define DRIVER_ADDRESS 0b00

SoftwareSerial swSer(DUMMY_RX_PIN, TX_PIN);     // RX, TX
TMC2209Stepper driver(&swSer, R_SENSE, DRIVER_ADDRESS);

/* ── motion parameters ───────────────────────────────────────────── */
const uint32_t DESIRED_SPEED = 1500;   // µsteps /s for gripper
const uint16_t MICROSTEPS    = 32;

bool tmcFwd = false, tmcBack = false;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) {}
  
//   swSer.begin(19200); // 19200 is highly stable for Uno SoftwareSerial
  
//   pinMode(EN_AB, OUTPUT);
//   digitalWrite(EN_AB, LOW); // ★ LOW = Driver ENABLED continuously for testing

//   // Push configs to the driver (Write-only)
//   driver.begin();
//   driver.pdn_disable(true);
//   driver.toff(4);
//   driver.en_spreadCycle(false);
//   driver.rms_current(700);       // Set to 0.7A
//   driver.mstep_reg_select(true);
//   driver.microsteps(MICROSTEPS);
//   driver.pwm_autoscale(true);

//   stepper3.setMaxSpeed(DESIRED_SPEED);
  
//   Serial.println(F("\n=== Gripper Test Mode ==="));
//   Serial.println(F("w  → Close"));
//   Serial.println(F("s  ← Open"));
//   Serial.println(F("z  ⏹ Stop"));
//   Serial.println(F("x  ⏹ Driver off"));
// }
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  swSer.begin(19200); 
  
  pinMode(EN_AB, OUTPUT);
  // 1. Keep power to the coils OFF during setup
  digitalWrite(EN_AB, HIGH); 

  // 2. Push configs to the driver safely
  driver.begin();
  driver.pdn_disable(true);
  driver.toff(4);
  driver.en_spreadCycle(false);
  driver.rms_current(700);       // Safe 0.7A limit established
  driver.mstep_reg_select(true);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(true);

  stepper3.setMaxSpeed(DESIRED_SPEED);
  
  Serial.println(F("\n=== Gripper Test Mode ==="));
  Serial.println(F("w  → Close"));
  Serial.println(F("s  ← Open"));
  Serial.println(F("z  ⏹ Stop"));
  Serial.println(F("x  ⏹ Driver off"));

  // 3. (Optional) Turn the driver ON now that it's safely configured, 
  // or just wait for the first 'w' or 's' command to wake it up.
  // digitalWrite(EN_AB, LOW); 
}
void loop() {
  /* 1. parse serial input ------------------------------------------ */
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Ignore newline characters without breaking the loop
    if (cmd != '\n' && cmd != '\r') {
      
      tmcFwd  = tmcBack  = false; // Clear previous states

      switch (cmd) {
      case 'w': 
        Serial.println("Closing...");      
        tmcFwd = true;
        digitalWrite(EN_AB, LOW);
        break;

      case 's': 
        Serial.println("Opening...");       
        tmcBack = true;
        digitalWrite(EN_AB, LOW);
        break;

      case 'z': 
        Serial.println("Stopping...");
        // Both flags are false, speed will be 0
        break;
      case 'x':
        Serial.println("Driver Disabled... (Freewheeling)");
        digitalWrite(EN_AB, HIGH); // Kill power to the coils
        break;

      default: 
        Serial.println("Unknown command.");
        break;
      }
    }
  }

  /* 2. Determine speeds ---------------------------------------------- */
  int32_t sp3 = 0;      // default: stop

  if (tmcFwd) { 
    sp3 = DESIRED_SPEED; 
  } else if (tmcBack) { 
    sp3 = -DESIRED_SPEED; 
  } 

  /* 3. push speeds and run ------------------------------------------ */
  stepper3.setSpeed(sp3);
  stepper3.runSpeed();   // Always process steps
}