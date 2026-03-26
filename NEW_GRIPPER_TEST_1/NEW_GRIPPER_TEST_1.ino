#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

const uint8_t STEP1 = 4,  DIR1 = 3;             // orientation-left
const uint8_t EN_AB = 11;                       // enable for pair
const uint8_t PDN_PIN = 9;

AccelStepper stepper3(AccelStepper::DRIVER, STEP1, DIR1);
#define R_SENSE        0.11f
#define DRIVER_ADDRESS 0b00
SoftwareSerial swSer(PDN_PIN, PDN_PIN);         // half-duplex on D9
TMC2209Stepper driver(&swSer, R_SENSE, DRIVER_ADDRESS);


/* ── motion parameters ───────────────────────────────────────────── */
const float    STEPPER_SPEED = 500;    // steps /s for orientation motors
const uint32_t DESIRED_SPEED = 1500;   // µsteps /s for gripper
const uint16_t MICROSTEPS    = 32;
bool tmcFwd   = false, tmcBack  = false;



void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while (!Serial) {}
  swSer.begin(38400);
  pinMode(EN_AB, OUTPUT);
  digitalWrite(EN_AB, HIGH); 

  driver.begin();
  driver.pdn_disable(true);
  driver.toff(4);
  driver.en_spreadCycle(false);
  driver.rms_current(700);
  driver.mstep_reg_select(true);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(true);

  /* gripper stepper ------------------------------------------------- */
  stepper3.setMaxSpeed(DESIRED_SPEED);
  // ── print key map ───────────────────────────────────────────────
  Serial.println(F("\n=== Control Keys ==="));
  Serial.println(F("w  → Moves UP"));
  Serial.println(F("s  ← Moves Down"));
  Serial.println();

  // Serial.print("IOIN: 0x"); Serial.println(driver.IOIN(), HEX);
  // Serial.print("GSTAT: 0x"); Serial.println(driver.GSTAT(), HEX);
  // Serial.print("Version: "); Serial.println(driver.version());

}

void loop() {
  // put your main code here, to run repeatedly:
/* 1. parse serial input ------------------------------------------ */
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;   // ignore CR / LF

    /* clear previous intents */
    // diffFwd = diffBack = rotCW = rotCCW = false;
    tmcFwd  = tmcBack  = false;

    switch (cmd) {
    /* gripper keys -------------------------------------------------- */
    case 'w': Serial.println("TMC → close");      tmcFwd   = true;
      digitalWrite(EN_AB, LOW);                 // ★ turn chassis OFF
      // digitalWrite(EN_PIN, LOW);                 //  enable gripper
      break;

    case 's': Serial.println("TMC ← open");       tmcBack  = true;
      digitalWrite(EN_AB, LOW);                 // ★ turn chassis OFF
      // digitalWrite(EN_PIN, LOW);
      break;

    /* pause but HOLD orientation ----------------------------------- */
    case 'x': Serial.println("⏸ hold orientation, stop gripper");
      digitalWrite(EN_AB, HIGH);                  // hold chassis
      // digitalWrite(EN_PIN, HIGH);                // disable gripper
      break;

    /* full release -------------------------------------------------- */
    case 'z': Serial.println("⏹ release ALL");
      digitalWrite(EN_AB, HIGH);                 // orientation free
      // digitalWrite(EN_PIN, HIGH);                // gripper off
      break;

    case 'h':                                  // on-demand help
      Serial.println(F("\n=== Control Keys ==="));
      Serial.println(F("w/s : close/open gripper"));
      Serial.println(F("x   : pause (hold chassis)"));
      Serial.println(F("z   : release all"));
      break;

    default : Serial.println("unknown");        break;
    }
}


 int32_t  sp3 = 0;      // default: stop everything

  if (tmcFwd)   { sp3 =  DESIRED_SPEED; }
  else if (tmcBack)  { sp3 = -DESIRED_SPEED; }

  /* 3. push speeds -------------------------------------------------- */
  stepper3.setSpeed(sp3);

  /* 4. run only the active axes ------------------------------------ */
  // if (!(tmcFwd || tmcBack)) {          // ★ chassis runs ONLY when gripper idle
  //   stepper1.runSpeed();
  //   stepper2.runSpeed();
  // }
  stepper3.runSpeed();                 // gripper always serviced
}
