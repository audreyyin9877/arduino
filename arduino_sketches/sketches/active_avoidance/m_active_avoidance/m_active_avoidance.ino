/*
 * LeDoux Lab - DEC 2021
 * jose [dot] cruz [at] nyu [dot] edu
 * ay2376 [at] nyu [dot] edu
 * snm427 [at] nyu [dot] edu
 */


 /*
    LIBRARIES
 */
 #include <SharpIR.h>
 #include <Tone.h>


/*
    VARIABLES
*/
//##################################################################################################################
// EXPERIMENTAL VARIABLES
const int N_TRIALS = 20;
unsigned long ACCLIMATION_DURATION = 20;                       // SECONDS
unsigned long TONE_DURATION = 15;                              // SECONDS
unsigned long SHOCK_DURATION = 1;                              // SECONDS
int CS_FREQUENCY = 5000;                                       // IN HERTZ
int ITI_INTERVALS[] = {40, 60, 80, 100, 120};                  // list of the inter-trial-intervals: ITI SECONDS
unsigned long MOTION_DETECTION_DURATION = 30;                  // SECONDS
//##################################################################################################################
//##################################################################################################################
//##################################################################################################################
//##################################################################################################################
//##################################################################################################################
// CHECK SENSORS VARIABLES
// Reading arrays
const unsigned int _NUM_READINGS = 400;                        // How many readings from each sensor
bool TEST_PASS = true;

// Timing variables for yellow light blink
unsigned long YELLOW_LED_END_TIME = 0;
const long BLINK_INTERVAL = 1000;
int YELLOW_STATE = LOW;
//##################################################################################################################
// LOCATION VARIABLES
int LEFT_ACTIVE;                                               // HIGH IF A COMPARTMENT IS ACTIVE, ELSE LOW
int RIGHT_ACTIVE;
//##################################################################################################################
// TIMING VARIABLES
unsigned long CURRENT_TONE_DELAY;
unsigned long START_TONE;
unsigned long DELTA_TONE_SHOCK = TONE_DURATION - SHOCK_DURATION;
unsigned long ITI_DURATION;

// For control of motion detection
unsigned long MOTION_DETECTION_START;
unsigned long MOTION_DETECTION_CURR;
//##################################################################################################################
// VARIABLES FOR STATISTICS
unsigned long ESCAPE_LATENCY_START;
unsigned long ESCAPE_LATENCY_END;
unsigned long ESCAPE_LATENCY_DELTA;
float ESCAPE_LATENCY_CUMULATIVE;

// SESSION
int TOTAL_AVOIDANCE_SUCCESS = 0;                           // CUMULATIVE COUNT OF SUCCESSFUL AVOIDANCE RESPONSES
int TOTAL_AVOIDANCE_FAILURE = 0;                           // CUMULATIVE COUNT OF FAILED AVOIDANCE RESPONSES
//##################################################################################################################


/*
    PIN ASSIGNMENTS
*/
//##################################################################################################################
// DIGITAL PINS
// Speakers
const int speaker_pin = 3;

// Shockers
const int shocker_r_pin = 4;
const int shocker_l_pin = 5;

// Buzzers
const int buzzer_pin_r = 6;
const int buzzer_pin_l = 7;

// IR LED Lights
const int speaker_led_r = 9;
const int speaker_led_l = 10;

// LED Check Lights (for UX design)
const int check_red_LED = 47;
const int check_yellow_LED = 49;
const int check_green_LED = 51;
//##################################################################################################################
// ANALOG PINS
// Right Sensors
#define ir_right A0
#define ir_right2 A4
#define ir_right3 A8
#define ir_right4 A12

// Left Sensors
#define ir_left A1
#define ir_left2 A5
#define ir_left3 A9
#define ir_left4 A13
//##################################################################################################################


/*
    INITIALIZING LIBRARIES
*/
//##################################################################################################################
// TONE LIBRARY
Tone SPEAKER_RIGHT;
Tone SPEAKER_LEFT;
//##################################################################################################################
// IR SENSOR LIBRARY
#define model 1080

// Right Sensors
SharpIR IR_SENSOR_R1 = SharpIR(ir_right, model);
SharpIR IR_SENSOR_R2 = SharpIR(ir_right2, model);
SharpIR IR_SENSOR_R3 = SharpIR(ir_right3, model);
SharpIR IR_SENSOR_R4 = SharpIR(ir_right4, model);

// Left Sensors
SharpIR IR_SENSOR_L1 = SharpIR(ir_left, model);
SharpIR IR_SENSOR_L2 = SharpIR(ir_left2, model);
SharpIR IR_SENSOR_L3 = SharpIR(ir_left3, model);
SharpIR IR_SENSOR_L4 = SharpIR(ir_left4, model);

// SENSOR THRESHOLDS
const int IR_THRESHOLD = 20;

// SENSOR READINGS
int L1_READING; int L2_READING; int L3_READING; int L4_READING;
int R1_READING; int R2_READING; int R3_READING; int R4_READING;
//##################################################################################################################
// Analog readings from multiple pinds are very inaccurate. The trick is to read them twice, with a small dealy after each read
// (10ms tends is good), then discard the first reading. This is because the ADC multiplexier needs switching time and the voltage
// needs time to stabalize after switching. To see more, check: https://forum.arduino.cc/t/reading-multiple-analog-inputs/55019

// This class calls for a delay in ms without using a delay.
class Timer {
  // Class Member Variables
  long onTime;                        // milliseconds of delay being on
  unsigned long sensor_cal_start;     // will store the last time LED was updated

  public:
  Timer(long on){
    onTime = on;
    sensor_cal_start = 0;
  }

  void CheckDelay() {
    // check to see if it's time to stop the delay
    unsigned long sensor_cal_current = millis();

    while (sensor_cal_start - sensor_cal_current < onTime) {
            sensor_cal_start = millis();
    }
  }
};

// Create a Timer object.
Timer sensor_cal(10);
//##################################################################################################################

void setup() {

  // INITIATE SERIAL
  Serial.begin(9600);

  SPEAKER_RIGHT.begin(buzzer_pin_r);
  SPEAKER_LEFT.begin(buzzer_pin_l);

  pinMode(shocker_r_pin, OUTPUT);
  pinMode(shocker_l_pin, OUTPUT);

  pinMode(speaker_led_r, OUTPUT);
  pinMode(speaker_led_l, OUTPUT);

  pinMode(check_red_LED, OUTPUT);
  pinMode(check_yellow_LED, OUTPUT);
  pinMode(check_green_LED, OUTPUT);

  /*
  // TEST SENSORS PROTOCOL
  // uncomment when you want to test
  while (true){
    L1_READING = IR_SENSOR_L1.distance(); delay(10); L1_READING = IR_SENSOR_L1.distance();
    L2_READING = IR_SENSOR_L2.distance(); delay(10); L2_READING = IR_SENSOR_L2.distance();
    L3_READING = IR_SENSOR_L3.distance(); delay(10); L3_READING = IR_SENSOR_L3.distance();
    L4_READING = IR_SENSOR_L4.distance(); delay(10); L4_READING = IR_SENSOR_L4.distance();

    R1_READING = IR_SENSOR_R1.distance(); delay(10); R1_READING = IR_SENSOR_R1.distance();
    R2_READING = IR_SENSOR_R2.distance(); delay(10); R2_READING = IR_SENSOR_R2.distance();
    R3_READING = IR_SENSOR_R3.distance(); delay(10); R3_READING = IR_SENSOR_R3.distance();
    R4_READING = IR_SENSOR_R4.distance(); delay(10); R4_READING = IR_SENSOR_R4.distance();

    Serial.println("L1 L2 L3 L4 R1 R2 R3 R4");
    Serial.print(L1_READING); Serial.print(" ");
    Serial.print(L2_READING); Serial.print(" ");
    Serial.print(L3_READING); Serial.print(" ");
    Serial.print(L4_READING); Serial.print(" ");
    Serial.print(R1_READING); Serial.print(" ");
    Serial.print(R2_READING); Serial.print(" ");
    Serial.print(R3_READING); Serial.print(" ");
    Serial.print(R4_READING); Serial.print(" ");
    Serial.println();

    delay(2000);
  }
    /**/

  // CHECK SENSORS PROTOCOL
  // Print entry message
  delay(5000);
  Serial.println("***CHECK SENSOR READINGS***");
  Serial.println("IF THE LIGHT IS: ");
  Serial.print("\t"); Serial.println("SOLID RED >>> RESET THE BOARD");
  Serial.print("\t"); Serial.println("BLINKING YELLOW >>> BOARD IS CHECKING SENSORS");
  Serial.print("\t"); Serial.println("BLINKING RED >>> SENSOR CHECK HAS FAILED :(");
  Serial.print("\t"); Serial.println("GREEN >>> CONTINUE WITH THE EXPERIMENT :)");

  // Turn off LED lights except for reed
  digitalWrite(check_red_LED, HIGH);
  digitalWrite(check_yellow_LED, LOW);
  digitalWrite(check_green_LED, LOW);

  // Print message to user. Lets them know the computer is doing something
  Serial.println("COLLECTING AND EVALUATING SENSOR READINGS...");

  // COLLECT 400 SENSOR READINGS
  // Create empty arrays
  unsigned int CHECK_R1[_NUM_READINGS];
  unsigned int CHECK_R2[_NUM_READINGS];
  unsigned int CHECK_R3[_NUM_READINGS];
  unsigned int CHECK_R4[_NUM_READINGS];

  unsigned int CHECK_L1[_NUM_READINGS];
  unsigned int CHECK_L2[_NUM_READINGS];
  unsigned int CHECK_L3[_NUM_READINGS];
  unsigned int CHECK_L4[_NUM_READINGS];

  // Fill arrays
  for (int i = 0; i < _NUM_READINGS; i++){
    CHECK_R1[i] = IR_SENSOR_R1.distance(); delay(10); CHECK_R1[i] = IR_SENSOR_R1.distance();
    CHECK_R2[i] = IR_SENSOR_R2.distance(); delay(10); CHECK_R2[i] = IR_SENSOR_R2.distance();
    CHECK_R3[i] = IR_SENSOR_R3.distance(); delay(10); CHECK_R3[i] = IR_SENSOR_R3.distance();
    CHECK_R4[i] = IR_SENSOR_R4.distance(); delay(10); CHECK_R4[i] = IR_SENSOR_R4.distance();

    CHECK_L1[i] = IR_SENSOR_L1.distance(); delay(10); CHECK_L1[i] = IR_SENSOR_L1.distance();
    CHECK_L2[i] = IR_SENSOR_L2.distance(); delay(10); CHECK_L2[i] = IR_SENSOR_L2.distance();
    CHECK_L3[i] = IR_SENSOR_L3.distance(); delay(10); CHECK_L3[i] = IR_SENSOR_L3.distance();
    CHECK_L4[i] = IR_SENSOR_L4.distance(); delay(10); CHECK_L4[i] = IR_SENSOR_L4.distance();

    // Blink yellow LED
    unsigned long YELLOW_LED_START_TIME = millis();
    if (YELLOW_LED_START_TIME - YELLOW_LED_END_TIME >= BLINK_INTERVAL) {
      YELLOW_LED_END_TIME = YELLOW_LED_START_TIME;
      if (YELLOW_STATE == LOW){
        YELLOW_STATE = HIGH;
      } else{
        YELLOW_STATE = LOW;
      }
      digitalWrite(check_yellow_LED, YELLOW_STATE);
    }
  }

  // FIND MIN VALUES
  // Create min variables
  unsigned int MIN_R1 = CHECK_R1[0];
  unsigned int MIN_R2 = CHECK_R2[0];
  unsigned int MIN_R3 = CHECK_R3[0];
  unsigned int MIN_R4 = CHECK_R4[0];

  unsigned int MIN_L1 = CHECK_L1[0];
  unsigned int MIN_L2 = CHECK_L2[0];
  unsigned int MIN_L3 = CHECK_L3[0];
  unsigned int MIN_L4 = CHECK_L4[0];

  for (int i = 0; i < _NUM_READINGS; i++){
    // Documentation: https://www.arduino.cc/reference/en/language/functions/math/min/
    MIN_R1 = min(CHECK_R1[i], MIN_R1);
    MIN_R2 = min(CHECK_R2[i], MIN_R2);
    MIN_R3 = min(CHECK_R3[i], MIN_R3);
    MIN_R4 = min(CHECK_R4[i], MIN_R4);

    MIN_L1 = min(CHECK_L1[i], MIN_L1);
    MIN_L2 = min(CHECK_L2[i], MIN_L2);
    MIN_L3 = min(CHECK_L3[i], MIN_L3);
    MIN_L4 = min(CHECK_L4[i], MIN_L4);
  }

  // COMPARE MIN VALUES WITH IR THRESHOLDS SET
  unsigned int MIN_ARRAY[8] = {MIN_L1, MIN_L2, MIN_L3, MIN_L4,
                              MIN_R1, MIN_R2, MIN_R3, MIN_R4};

  for (int i = 0; i < (sizeof(MIN_ARRAY) / sizeof(MIN_ARRAY[0])); i++){
    if (MIN_ARRAY[i] < IR_THRESHOLD){
      TEST_PASS = false;
    }
  }

  // TURN ON LED LIGHT BASED ON CHECK SENSOR OUTCOME
  // If check sensor protocol passed
  if (TEST_PASS){
    // Message to user
    Serial.println("Sensor check complete! Continue with the experiment.");

    // Turn on LEDs
    digitalWrite(check_green_LED, HIGH);
    digitalWrite(check_yellow_LED, LOW);
    digitalWrite(check_red_LED, LOW);

  // if check sensor protcol failed. while(true) forces user to call for help
  } else if (!TEST_PASS){
    bool TEST_START = false;

    // Message to user
    Serial.println("Sensor check has failed. Please contact either Rodrigo or Audrey.");

    // Turn on LEDs
    digitalWrite(check_red_LED, LOW);
    digitalWrite(check_yellow_LED, LOW);
    int RED_STATE = LOW;
    while(true){
      if (RED_STATE == LOW){
        RED_STATE = HIGH;
      } else {
        RED_STATE = LOW;
      }
      digitalWrite(check_red_LED, RED_STATE);
      delay(300);

      // Recover minimum values from readings. Can input 2 on serial monitor, or Alt-T on Bonsai-Rx to see values
      int x = Serial.parseInt();
      if (x==2){
        TEST_START = true;
      }

      if (TEST_START){
        // Reset serial input from Bonsai
        x = 0;
        TEST_START = false;

        Serial.println("Minimum values: ");
        Serial.println("L1 L2 L3 L4 R1 R2 R3 R4");
        for (int i = 0; i < (sizeof(MIN_ARRAY) / sizeof(MIN_ARRAY[0])); i++){
          Serial.print(MIN_ARRAY[i]); Serial.print(" ");
        }
        Serial.println();

        Serial.println("Current sensor values: ");
        L1_READING = IR_SENSOR_L1.distance(); delay(10); L1_READING = IR_SENSOR_L1.distance();
        L2_READING = IR_SENSOR_L2.distance(); delay(10); L2_READING = IR_SENSOR_L2.distance();
        L3_READING = IR_SENSOR_L3.distance(); delay(10); L3_READING = IR_SENSOR_L3.distance();
        L4_READING = IR_SENSOR_L4.distance(); delay(10); L4_READING = IR_SENSOR_L4.distance();

        R1_READING = IR_SENSOR_R1.distance(); delay(10); R1_READING = IR_SENSOR_R1.distance();
        R2_READING = IR_SENSOR_R2.distance(); delay(10); R2_READING = IR_SENSOR_R2.distance();
        R3_READING = IR_SENSOR_R3.distance(); delay(10); R3_READING = IR_SENSOR_R3.distance();
        R4_READING = IR_SENSOR_R4.distance(); delay(10); R4_READING = IR_SENSOR_R4.distance();
        Serial.println("L1 L2 L3 L4 R1 R2 R3 R4");
        Serial.print(L1_READING); Serial.print(" ");
        Serial.print(L2_READING); Serial.print(" ");
        Serial.print(L3_READING); Serial.print(" ");
        Serial.print(L4_READING); Serial.print(" ");
        Serial.print(R1_READING); Serial.print(" ");
        Serial.print(R2_READING); Serial.print(" ");
        Serial.print(R3_READING); Serial.print(" ");
        Serial.print(R4_READING); Serial.print(" ");
        Serial.println();
      }
    }

  // if something goes completely wrong, yellow light turns on, and user is forced to call for help
  } else {
    digitalWrite(check_yellow_LED, HIGH);

    // Message to user
    Serial.println("Sensor check has failed. Please contact either Rodrigo or Audrey.");
    while(true) {

    }
  }


}

void loop() {

  bool SESSION_START = false;
  bool TEST_START = false;

  // TRIGGER START OF TRIAL VIA BONSAI-RX
  int x = Serial.parseInt();
  if (x == 1) {
    SESSION_START = true;
  } else if (x == 2) {
    TEST_START = true;
  } else {
    // NOTHING
  }


  // TEST CHAMBER COMPONENTS
  if (TEST_START) {

    // RESET SERIAL INPUT FROM BONSAI-RX
    x = 0;
    TEST_START = false;

    // TEST LEDs
    Serial.println("TEST CHAMBER LEDs");
    delay(500);
    digitalWrite(speaker_led_r, HIGH);
    digitalWrite(speaker_led_l, HIGH);
    delay(3000);
    digitalWrite(speaker_led_r, LOW);
    digitalWrite(speaker_led_l, LOW);
    Serial.println("TEST CHAMBER LEDs > COMPLETE");
    delay(500);

    // TEST TONE GENERATION
    Serial.println("TEST TONE GENERATION");
    delay(500);
    SPEAKER_RIGHT.play(CS_FREQUENCY);
    SPEAKER_LEFT.play(CS_FREQUENCY);
    delay(3000);
    SPEAKER_RIGHT.stop();
    SPEAKER_LEFT.stop();
    Serial.println("TEST TONE GENERATION > COMPLETE");
    delay(500);

    // TEST SHOCK GENERATION LEFT, THEN RIGHT
    Serial.println("TEST SHOCKER");
    delay(500);
    Serial.println("RIGHT SIDE");
    digitalWrite(shocker_r_pin, HIGH);
    delay(3000);
    digitalWrite(shocker_r_pin, LOW);
    Serial.println("LEFT SIDE");
    digitalWrite(shocker_l_pin, HIGH);
    delay(3000);
    digitalWrite(shocker_l_pin, LOW);
    Serial.println("TEST SHOCKER > COMPLETE");
    delay(500);

    // TEST PIR SENSOR LEFT, THEN RIGHT

  }

  // RESET CUMMULATIVE VARIABLE VALUES
  TOTAL_AVOIDANCE_FAILURE = 0;
  TOTAL_AVOIDANCE_SUCCESS = 0;
  ESCAPE_LATENCY_CUMULATIVE = 0;


  // START EXPERIMENT SESSION
  if (SESSION_START) {

    // RESET SERIAL INPUT FROM BONSAI-RX
    x = 0;
    SESSION_START = false;


    // PRINT BASIC SESSION INFORMATION
    // ###############################
    Serial.println("ACTIVE AVOIDANCE");
    Serial.print("NUMBER OF TRIALS: "); Serial.println(N_TRIALS);
    Serial.print("TONE DURATION (SEC): "); Serial.println(TONE_DURATION);
    Serial.print("SHOCK DURATION (SEC): "); Serial.println(SHOCK_DURATION);
    Serial.print("TONE PAIRED WITH SHOCK AT (SEC): "); Serial.println(TONE_DURATION - SHOCK_DURATION);

    // SIGNAL START OF THE SESSION
    for (int x = 0; x < 5; x ++) {

      // TURN LED ON IN BOTH SIDES
      digitalWrite(speaker_led_r, HIGH); digitalWrite(speaker_led_l, HIGH);
      delay(500);

      // TURN LED OFF IN BOTH SIDES
      digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);
      delay(500);
    }
    Serial.println("SESSION > START");

    // ACCLIMATION PERIOD
    // ##################
    Serial.print("ACCLIMATION (SEC): "); Serial.println(ACCLIMATION_DURATION);
    delay(ACCLIMATION_DURATION*1000);

    // START TRIALS
    for (int x = 0; x < N_TRIALS; x++) {

      // PRINT INFORMATION ABOUT TRIAL
      Serial.print("TRIAL NUMBER "); Serial.print(x+1); Serial.println(" > START");


      // TRIAL ONE UNAVOID
      if (x == 0) {

          // TURN THE SPEAKER ON
          SPEAKER_RIGHT.play(CS_FREQUENCY);                        // FREQUENCY
          SPEAKER_LEFT.play(CS_FREQUENCY);                         // FREQUENCY
          Serial.println("CS > ON");

          // TURN LED ON IN BOTH SIDES
          digitalWrite(speaker_led_r, HIGH); digitalWrite(speaker_led_l, HIGH);

          // RECORD START OF THE TONE
          START_TONE = millis();

        while (true) {

          CURRENT_TONE_DELAY = millis();

          if ((CURRENT_TONE_DELAY - START_TONE) > (DELTA_TONE_SHOCK * 1000)) {

            // TRIGGER US
            digitalWrite(shocker_l_pin, HIGH);
            Serial.println("US_L > ON");
            digitalWrite(shocker_r_pin, HIGH);
            Serial.println("US_R > ON");

            // KEEP US FOR SPECIFIC TIME DELAY
            for (int i = 0; i < SHOCK_DURATION; i++) {
              delay(1000);
            }

            // TERMINATE SHOCKER
            digitalWrite(shocker_l_pin, LOW);
            Serial.println("US_L > OFF");
            digitalWrite(shocker_r_pin, LOW);
            Serial.println("US_R > OFF");

            // TERMINATE TONE IN THE COMPARTMENT IF AFTER SHOCK
            SPEAKER_RIGHT.stop();
            SPEAKER_LEFT.stop();
            Serial.println("CS > OFF");

            // RECORD LATENCY_END WHEN NO SHUTTLING
            ESCAPE_LATENCY_END = 0;

            // COUNT ONE TOWARDS AVOIDANCE FAILURE
            TOTAL_AVOIDANCE_FAILURE ++;

            // TURN LED OFF IN BOTH SIDES
            digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);

            break;
          }

        }

        // SELECT RANDOM ITI FROM LIST
        ITI_DURATION = ITI_INTERVALS[random(0, 5)];
        Serial.print("INTER-TRIAL-INTERVAL (SEC): "); Serial.println(ITI_DURATION);

        // INITIATE INTER-TRIAL-INTERVAL
        delay(ITI_DURATION*1000);

        // PRINT EXIT INFORMATION ABOUT TRIAL
        // ###########################################################################
        Serial.print("TRIAL "); Serial.print(x+1); Serial.println(" > END");
        Serial.print("TRIAL "); Serial.print(x+1); Serial.println(" ESCAPE LATENCY (SEC): 0");  // FIRST IS UNAVOID
        Serial.print("CUMULATIVE TOTAL SHUTTLINGS: "); Serial.println(TOTAL_AVOIDANCE_SUCCESS);
        Serial.print("CUMULATIVE TOTAL FAILURES: "); Serial.println(TOTAL_AVOIDANCE_FAILURE);

        // RESET ACTIVE CHAMBER SIDE SENSOR VARIABLES
        RIGHT_ACTIVE = LOW;
        LEFT_ACTIVE = LOW;

        // CONTINUE TO THE NEXT TRIAL
        continue;

      }

      // START TIME FOR MOTION DETECTION FEATURE
      MOTION_DETECTION_START = millis();

      // DETECT POSITION, DELIVER CS AND US
      while (true) {
        L1_READING = IR_SENSOR_L1.distance(); sensor_cal.CheckDelay(); L1_READING = IR_SENSOR_L1.distance();
        L2_READING = IR_SENSOR_L2.distance(); sensor_cal.CheckDelay(); L2_READING = IR_SENSOR_L2.distance();
        L3_READING = IR_SENSOR_L3.distance(); sensor_cal.CheckDelay(); L3_READING = IR_SENSOR_L3.distance();
        L4_READING = IR_SENSOR_L4.distance(); sensor_cal.CheckDelay(); L4_READING = IR_SENSOR_L4.distance();

        R1_READING = IR_SENSOR_R1.distance(); sensor_cal.CheckDelay(); R1_READING = IR_SENSOR_R1.distance();
        R2_READING = IR_SENSOR_R2.distance(); sensor_cal.CheckDelay(); R2_READING = IR_SENSOR_R2.distance();
        R3_READING = IR_SENSOR_R3.distance(); sensor_cal.CheckDelay(); R3_READING = IR_SENSOR_R3.distance();
        R4_READING = IR_SENSOR_R4.distance(); sensor_cal.CheckDelay(); R4_READING = IR_SENSOR_R4.distance();

        if (R1_READING < IR_THRESHOLD ||
        R2_READING < IR_THRESHOLD ||
        R3_READING < IR_THRESHOLD ||
        R4_READING < IR_THRESHOLD) {
          RIGHT_ACTIVE = HIGH;
          LEFT_ACTIVE = LOW;
        } else if (L1_READING < IR_THRESHOLD ||
        L2_READING < IR_THRESHOLD ||
        L3_READING < IR_THRESHOLD ||
        L4_READING < IR_THRESHOLD) {
          LEFT_ACTIVE = HIGH;
          RIGHT_ACTIVE = LOW;
        } else {
          RIGHT_ACTIVE = LOW;
          LEFT_ACTIVE = LOW;
          // END TIME FOR MOTION DETECTION FEATURE
          MOTION_DETECTION_CURR = millis();
        }

        // RESET VARIABLES FOR SHUTTLING
        ESCAPE_LATENCY_START = 0;
        ESCAPE_LATENCY_END = 0;

        if (RIGHT_ACTIVE) {

          // DELIVER TONE IN THE RIGHT COMPARTMENT
          Serial.println("RIGHT COMPARTMENT > ACTIVE");
          SPEAKER_RIGHT.play(CS_FREQUENCY);
          SPEAKER_LEFT.play(CS_FREQUENCY);
          Serial.println("CS > ON");

          // RECORD LATENCY_START
          ESCAPE_LATENCY_START = millis();

          // TURN LED ON IN BOTH SIDES
          digitalWrite(speaker_led_r, HIGH); digitalWrite(speaker_led_l, HIGH);

          START_TONE = millis();
          CURRENT_TONE_DELAY = millis();

          // ADD 0.5 SEC DELAY TO AVOID SENSOR DETECTION ARTIFACTS
          unsigned long S_DELAY_START = millis();
          unsigned long S_DELAY_CURRENT = millis();
          while (S_DELAY_START - S_DELAY_CURRENT < 500) {
            S_DELAY_CURRENT = millis();
          }

          // WHILE THE OPPOSITE COMPARTMENT IS NOT ACTIVE, CONTINUE FOR TONE_DURATION
          while (true) {
            L1_READING = IR_SENSOR_L1.distance(); sensor_cal.CheckDelay(); L1_READING = IR_SENSOR_L1.distance();
            L2_READING = IR_SENSOR_L2.distance(); sensor_cal.CheckDelay(); L2_READING = IR_SENSOR_L2.distance();

            // CHECK IF LEFT IS ACTIVE
            if (L1_READING < IR_THRESHOLD ||
            L2_READING < IR_THRESHOLD) {
              LEFT_ACTIVE = HIGH;
              RIGHT_ACTIVE = LOW;
            } else {
              LEFT_ACTIVE = LOW;
              RIGHT_ACTIVE = LOW;
            }

            CURRENT_TONE_DELAY = millis();

            if (LEFT_ACTIVE == HIGH) {

              // IF THE LEFT IS HIGH THEN TERMINATE TONE AND MOVE TO ITI
              // TERMINATES TONE IN THE RIGHT COMPARTMENT
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // RECORD LATENCY_END WHEN SHUTTLING
              ESCAPE_LATENCY_END = millis();

              // CUMULATIVE COUNT OF THE LATENCY TO CALCULATE THE MEAN AT THE END OF THE SESSION
              ESCAPE_LATENCY_CUMULATIVE = ESCAPE_LATENCY_CUMULATIVE + ((ESCAPE_LATENCY_END - ESCAPE_LATENCY_START)/1000);

              // COUNT ONE TOWARDS AVOIDANCE SUCCESS
              TOTAL_AVOIDANCE_SUCCESS ++;

              // TURN LED OFF IN BOTH SIDES
              digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);

              // RESET LEFT SENSOR VALUE
              LEFT_ACTIVE = LOW;

              // CONTINUE TO THE NEXT TRIAL
              break;
            }

            // AFTER SPECIFIC DELAY, TRIGGER US
            if ((CURRENT_TONE_DELAY - START_TONE) > (DELTA_TONE_SHOCK * 1000)) {

              // TRIGGER US
              digitalWrite(shocker_r_pin, HIGH);
              Serial.println("US_R > ON");

              // KEEP US FOR SPECIFIC TIME DELAY
              for (int i = 0; i < SHOCK_DURATION; i++) {
                delay(1000);
              }

              // TERMINATE SHOCK IN THE RIGHT COMPARTMENT
              digitalWrite(shocker_r_pin, LOW);
              Serial.println("US_R > OFF");

              // TERMINATE TONE IN THE RIGHT COMPARTMENT IF AFTER SHOCK
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // RECORD LATENCY_END WHEN NO SHUTTLING
              ESCAPE_LATENCY_END = ESCAPE_LATENCY_START;

              // COUNT ONE TOWARDS AVOIDANCE FAILURE
              TOTAL_AVOIDANCE_FAILURE ++;

              // TURN LED OFF IN BOTH SIDES
              digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);
              break;
            }
          }

          break;

        } else if (LEFT_ACTIVE) {

          // DELIVER TONE IN THE LEFT COMPARTMENT
          Serial.println("LEFT COMPARTMENT > ACTIVE");
          SPEAKER_RIGHT.play(CS_FREQUENCY);
          SPEAKER_LEFT.play(CS_FREQUENCY);
          START_TONE = millis();
          CURRENT_TONE_DELAY = millis();
          Serial.println("CS > ON");

          // RECORD LATENCY_START
          ESCAPE_LATENCY_START = millis();

          // TURN LED ON IN BOTH SIDES
          digitalWrite(speaker_led_r, HIGH); digitalWrite(speaker_led_l, HIGH);

          // ADD 0.5 SEC DELAY TO AVOID SENSOR DETECTION ARTIFACTS
          unsigned long S_DELAY_START = millis();
          unsigned long S_DELAY_CURRENT = millis();
          while (S_DELAY_START - S_DELAY_CURRENT < 500) {
            S_DELAY_CURRENT = millis();
          }

          // WHILE THE OPPOSITE COMPARTMENT IS NOT ACTIVE, CONTINUE FOR TONE_DURATION
          while (true) {
            R3_READING = IR_SENSOR_R3.distance(); sensor_cal.CheckDelay(); R3_READING = IR_SENSOR_R3.distance();
            R4_READING = IR_SENSOR_R4.distance(); sensor_cal.CheckDelay(); R4_READING = IR_SENSOR_R4.distance();

            // CHECK IF RIGHT IS ACTIVE
            if (R3_READING < IR_THRESHOLD ||
            R4_READING < IR_THRESHOLD) {
              RIGHT_ACTIVE = HIGH;
              LEFT_ACTIVE = LOW;
            } else {
              RIGHT_ACTIVE = LOW;
              LEFT_ACTIVE = LOW;
            }

            if (RIGHT_ACTIVE == HIGH) {

              // IF THE RIGHT IS HIGH THEN TERMINATE TONE AND MOVE TO ITI
              // TERMINATES TONE
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // RECORD LATENCY_END WHEN SHUTTLING
              ESCAPE_LATENCY_END = millis();

              // CUMULATIVE COUNT OF THE LATENCY TO CALCULATE THE MEAN AT THE END OF THE SESSION
              ESCAPE_LATENCY_CUMULATIVE = ESCAPE_LATENCY_CUMULATIVE + ((ESCAPE_LATENCY_END - ESCAPE_LATENCY_START)/1000);

              // COUNT ONE TOWARDS AVOIDANCE SUCCESS
              TOTAL_AVOIDANCE_SUCCESS ++;

              // TURN LED OFF IN BOTH SIDES
              digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);

              // RESET RIGHT SENSOR VALUE
              RIGHT_ACTIVE == LOW;

              // CONTINUE TO THE NEXT TRIAL
              break;
            }

            // AFTER SPECIFIC DELAY, TRIGGER US
            if ((CURRENT_TONE_DELAY - START_TONE) > (DELTA_TONE_SHOCK * 1000)) {

              // TRIGGER US
              digitalWrite(shocker_l_pin, HIGH);
              Serial.println("US_L > ON");

              // KEEP US FOR SPECIFIC TIME DELAY
              for (int i = 0; i < SHOCK_DURATION; i++) {
                delay(1000);
              }

              // TERMINATE SHOCKER
              digitalWrite(shocker_l_pin, LOW);
              Serial.println("US_L > OFF");

              // TERMINATE TONE IN THE COMPARTMENT IF AFTER SHOCK
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // RECORD LATENCY_END WHEN NO SHUTTLING
              ESCAPE_LATENCY_END = ESCAPE_LATENCY_START;

              // COUNT ONE TOWARDS AVOIDANCE FAILURE
              TOTAL_AVOIDANCE_FAILURE ++;

              // TURN LED OFF IN BOTH SIDES
              digitalWrite(speaker_led_r, LOW); digitalWrite(speaker_led_l, LOW);

              break;
            }
            CURRENT_TONE_DELAY = millis();
          }
          break;


        } else if (!LEFT_ACTIVE && !RIGHT_ACTIVE
                   && (MOTION_DETECTION_CURR - MOTION_DETECTION_START) >= (MOTION_DETECTION_DURATION * 1000)){
            // SERIAL OUTPUT MESSAGE TO USER
            Serial.println("MOTION DETECTION FAILED");
            Serial.print("NO MOTION DETECTED IN "); Serial.print(MOTION_DETECTION_DURATION); Serial.println(" SECONDS");

            // TRIGGER US
            digitalWrite(shocker_l_pin, HIGH);
            Serial.println("US_L > ON");
            digitalWrite(shocker_r_pin, HIGH);
            Serial.println("US_R > ON");

            // KEEP US FOR SPECIFIC TIME DELAY
            for (int i = 0; i < SHOCK_DURATION; i++) {
              delay(1000);
            }

            // TERMINATE SHOCKER
            digitalWrite(shocker_l_pin, LOW);
            Serial.println("US_L > OFF");
            digitalWrite(shocker_r_pin, LOW);
            Serial.println("US_R > OFF");

            // RECORD LATENCY_END WHEN NO SHUTTLING
            ESCAPE_LATENCY_END = 0;

            // COUNT ONE TOWARDS AVOIDANCE FAILURE
            TOTAL_AVOIDANCE_FAILURE ++;

            break;
        }

      }

      // SELECT RANDOM ITI FROM LIST
      ITI_DURATION = ITI_INTERVALS[random(0, 5)];
      Serial.print("INTER-TRIAL-INTERVAL (SEC): "); Serial.println(ITI_DURATION);

      // INITIATE INTER-TRIAL-INTERVAL
      delay(ITI_DURATION*1000);


      // PRINT EXIT INFORMATION ABOUT TRIAL
      // ###########################################################################
      Serial.print("TRIAL "); Serial.print(x+1); Serial.println(" > END");
      Serial.print("TRIAL "); Serial.print(x+1); Serial.print(" ESCAPE LATENCY (SEC): "); Serial.println((ESCAPE_LATENCY_END - ESCAPE_LATENCY_START)/1000.0);
      Serial.print("CUMULATIVE TOTAL SHUTTLINGS: "); Serial.println(TOTAL_AVOIDANCE_SUCCESS);
      Serial.print("CUMULATIVE TOTAL FAILURES: "); Serial.println(TOTAL_AVOIDANCE_FAILURE);

    }

    // SESSION FINAL INFO AND STATISTICS
    // ###########################################################################
    Serial.println("SESSION > END");
    Serial.println("SESSION STATISTICS");
    Serial.print("SESSION TOTAL SHUTTLINGS: "); Serial.println(TOTAL_AVOIDANCE_SUCCESS);
    Serial.print("SESSION TOTAL FAILURES: "); Serial.println(TOTAL_AVOIDANCE_FAILURE);
    if (TOTAL_AVOIDANCE_SUCCESS == 0) {
      Serial.print("SESSION MEAN SHUTTLING LATENCY (SEC): "); Serial.println(TOTAL_AVOIDANCE_SUCCESS);
    } else {
      Serial.print("SESSION MEAN SHUTTLING LATENCY (SEC): "); Serial.println((float)ESCAPE_LATENCY_CUMULATIVE / (float)TOTAL_AVOIDANCE_SUCCESS);
    }

  // CHANGE LED TO INDICATE RESET STATUS
  digitalWrite(check_green_LED, LOW);
  digitalWrite(check_red_LED, HIGH);

  while(true){
    // DO NOTHING. CAUSES PROGRAM TO ENTER AN ENDLESS LOOP AND STALL, FORCING USER TO RESTART
  }

  }

}
