/*
 * LeDoux Lab - DEC 2021
 * jose [dot] cruz [at] nyu [dot] edu
 * ay2376 [at] nyu [dot] edu
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
const int N_TRIALS = 20;
unsigned long ACCLIMATION_DURATION = 200;                       // SECONDS
unsigned long TONE_DURATION = 15;                              // SECONDS
unsigned long SHOCK_DURATION = 1;                              // SECONDS
int CS_FREQUENCY = 5000;                                       // IN HERTZ
int ITI_INTERVALS[] = {40, 60, 80, 100, 120};                  // list of the inter-trial-intervals: ITI
unsigned long MOTION_DETECTION_DURATION = 30;                  // SECONDS
const int OPTO_FULL_DURATION = 15;                              // SECONDS
const int OPTO_FLICKER_DURATION = 25;                          // IN HERTZ. Due to lag, the true frequency is 20Hz
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
int LEFT_ACTIVE;                                        // HIGH IF A COMPARTMENT IS ACTIVE, ELSE LOW
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
// OPTOGENETICS VARIABLES
// Timing variables for optogenetics
unsigned long OPTO_START;
unsigned long OPTO_CURRENT;
int OPTO_STATE = LOW;

unsigned long OPTO_FLICKER_START;
unsigned long OPTO_FLICKER_PREVIOUS = 0;

unsigned long OPTO_START_TIMESTAMP;
unsigned long OPTO_END_TIMESTAMP;
//##################################################################################################################
// VARIABLES FOR STATISTICS
unsigned long ESCAPE_LATENCY_START;
unsigned long ESCAPE_LATENCY_END;
unsigned long ESCAPE_LATENCY_DELTA;
float ESCAPE_LATENCY_CUMULATIVE;

// SESSION
int TOTAL_AVOIDANCE_SUCCESS = 0;                           // CUMULATIVE COUNT OF SUCCESSFUL AVOIDANCE RESPONSES
int TOTAL_AVOIDANCE_FAILURE = 0;                           // CUMULATIVE COUNT OF FAILED AVOIDANCE RESPONSES


/*
    PIN ASSIGNMENTS
*/
//##################################################################################################################
// DIGITAL PINS
// Shockers
const int shocker_r_pin = 4;
const int shocker_l_pin = 5;

// Buzzers
const int buzzer_pin_r = 6;
const int buzzer_pin_l = 7;

// IR LED lights
const int speaker_led_r = 9;
const int speaker_led_l = 10;

// LED Check Lights (for UX design)
const int check_red_LED = 23;
const int check_yellow_LED = 25;
const int check_green_LED = 27;

// Optogenetics
const int opto_LED = 12;

//fiber photometry pins
const int fiber_clock = 43;
const int fiber_trial = 45;
//##################################################################################################################
// ANALOG PINS
// Right Sensors
#define ir_right1 A0
#define ir_left1 A1

// Left Sensors
#define ir_right2 A2
#define ir_left2 A3
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
SharpIR IR_SENSOR_R1 = SharpIR(ir_right1, model);
SharpIR IR_SENSOR_L1 = SharpIR(ir_left1, model);

// Left Sensors
SharpIR IR_SENSOR_R2 = SharpIR(ir_right2, model);
SharpIR IR_SENSOR_L2 = SharpIR(ir_left2, model);

// SENSOR THRESHOLDS
int IF_THRESHOLD = 20;                                   // CM > DISTANCE FROM SENSOR TO OPPOSITE WALL.

// SENSOR READINGS
int L1_READING; int L2_READING;
int R1_READING; int R2_READING;
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

  // ASSIGN PINS
  SPEAKER_RIGHT.begin(buzzer_pin_r);
  SPEAKER_LEFT.begin(buzzer_pin_l);

  pinMode(shocker_r_pin, OUTPUT);
  pinMode(shocker_l_pin, OUTPUT);

  pinMode(speaker_led_r, OUTPUT);
  pinMode(speaker_led_l, OUTPUT);

  pinMode(check_red_LED, OUTPUT);
  pinMode(check_yellow_LED, OUTPUT);
  pinMode(check_green_LED, OUTPUT);

  pinMode(opto_LED, OUTPUT);

  pinMode(fiber_clock, OUTPUT);
  pinMode(fiber_trial, OUTPUT);

  /*
  // UNCOMMENT TO TEST SENSORS
  while (true) {
    L1_READING = IR_SENSOR_L1.distance(); delay(10); L1_READING = IR_SENSOR_L1.distance();
    L2_READING = IR_SENSOR_L2.distance(); delay(10); L2_READING = IR_SENSOR_L2.distance();
    R1_READING = IR_SENSOR_R1.distance(); delay(10); R1_READING = IR_SENSOR_R1.distance();
    R2_READING = IR_SENSOR_R2.distance(); delay(10); R2_READING = IR_SENSOR_R2.distance();

    Serial.println("L1 L2 R1 R2");
    Serial.print(L1_READING); Serial.print(" ");
    Serial.print(L2_READING); Serial.print(" ");
    Serial.print(R1_READING); Serial.print(" ");
    Serial.print(R2_READING); Serial.print(" ");
    Serial.println();

    delay(1000);
  }
  /**/

  // PRINT ENTRY MESSAGE
  delay(5000);
  Serial.println("***CHECK SENSOR READINGS***");
  Serial.println("IF THE LIGHT IS: ");
  Serial.print("\t"); Serial.println("SOLID RED >>> RESET THE BOARD");
  Serial.print("\t"); Serial.println("BLINKING YELLOW >>> BOARD IS CHECKING SENSORS");
  Serial.print("\t"); Serial.println("BLINKING RED >>> SENSOR CHECK HAS FAILED :(");
  Serial.print("\t"); Serial.println("GREEN >>> CONTINUE WITH THE EXPERIMENT :)");

  // CHECK Sensors
  // Turn off LED lights except for reed
  digitalWrite(check_red_LED, HIGH);
  digitalWrite(check_yellow_LED, LOW);
  digitalWrite(check_green_LED, LOW);

  // PRINT MESSAGE TO USER
  Serial.println("COLLECTING AND EVALUATING SENSOR READINGS...");

  // COLLECT 400 SENSOR readings
  // Create empty arrays
  unsigned int CHECK_R1[_NUM_READINGS];
  unsigned int CHECK_R2[_NUM_READINGS];

  unsigned int CHECK_L1[_NUM_READINGS];
  unsigned int CHECK_L2[_NUM_READINGS];

  // Fill arrays
  for (int i = 0; i < _NUM_READINGS; i++){
    CHECK_R1[i] = IR_SENSOR_R1.distance(); delay(10); CHECK_R1[i] = IR_SENSOR_R1.distance();
    CHECK_R2[i] = IR_SENSOR_R2.distance(); delay(10); CHECK_R2[i] = IR_SENSOR_R2.distance();

    CHECK_L1[i] = IR_SENSOR_L1.distance(); delay(10); CHECK_L1[i] = IR_SENSOR_L1.distance();
    CHECK_L2[i] = IR_SENSOR_L2.distance(); delay(10); CHECK_L2[i] = IR_SENSOR_L2.distance();

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

  unsigned int MIN_L1 = CHECK_L1[0];
  unsigned int MIN_L2 = CHECK_L2[0];

  for (int i = 0; i < _NUM_READINGS; i++){
    // Documentation: https://www.arduino.cc/reference/en/language/functions/math/min/
    MIN_R1 = min(CHECK_R1[i], MIN_R1);
    MIN_R2 = min(CHECK_R2[i], MIN_R2);

    MIN_L1 = min(CHECK_L1[i], MIN_L1);
    MIN_L2 = min(CHECK_L2[i], MIN_L2);
  }

  // COMPARE MIN VALUES WITH IR THRESHOLDS SET
  unsigned int MIN_ARRAY[4] = {MIN_L1, MIN_L2,
                              MIN_R1, MIN_R2};

  for (int i = 0; i < (sizeof(MIN_ARRAY) / sizeof(MIN_ARRAY[0])); i++){
    // The minimum found using 400 values is always greater than the true minimum.
    // We subtract 2 from the minimum found using 400 values to approximate the true minimum.
    MIN_ARRAY[i] -= 2;
    if (MIN_ARRAY[i] < IF_THRESHOLD){
      TEST_PASS = false;
    }
  }

  // TURN ON LED LIGHT BASED ON CHECK SENSOR OUTCOME
  if (TEST_PASS){
    // Message to user
    Serial.println("Sensor check complete! Continue with the experiment.");

    // Turn on LEDs
    digitalWrite(check_green_LED, HIGH);
    digitalWrite(check_yellow_LED, LOW);
    digitalWrite(check_red_LED, LOW);

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

      // Recover minimum values from readings
      int x = Serial.parseInt();
      if (x==2){
        TEST_START = true;
      }

      if (TEST_START){
        // Reset serial input from Bonsai
        x = 0;
        TEST_START = false;

        Serial.println("Minimum values calculated: ");
        Serial.println("L1 L2 R1 R2");
        for (int i = 0; i < (sizeof(MIN_ARRAY) / sizeof(MIN_ARRAY[0])); i++){
          Serial.print(MIN_ARRAY[i]); Serial.print(" ");
        }
        Serial.println();

        Serial.println("Current sensor values: ");
        L1_READING = IR_SENSOR_L1.distance(); delay(10); L1_READING = IR_SENSOR_L1.distance();
        L2_READING = IR_SENSOR_L2.distance(); delay(10); L2_READING = IR_SENSOR_L2.distance();
        R1_READING = IR_SENSOR_R1.distance(); delay(10); R1_READING = IR_SENSOR_R1.distance();
        R2_READING = IR_SENSOR_R2.distance(); delay(10); R2_READING = IR_SENSOR_R2.distance();
        Serial.println("L1 L2 R1 R2");
        Serial.print(L1_READING); Serial.print(" ");
        Serial.print(L2_READING); Serial.print(" ");
        Serial.print(R1_READING); Serial.print(" ");
        Serial.print(R2_READING); Serial.print(" ");
        Serial.println();
      }
    }

  } else {
    digitalWrite(check_yellow_LED, HIGH);
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


  // TEST CHAMBER
  if (TEST_START) {

    // RESET SERIAL INPUT FROM BONSAI-RX
    x = 0;
    TEST_START = false;

    // TEST LEDs
    Serial.println("TEST CHAMBER LEDs");
    delay(500);
    digitalWrite(speaker_led_r, HIGH);
    digitalWrite(speaker_led_l, HIGH);
    delay(5000);
    digitalWrite(speaker_led_r, LOW);
    digitalWrite(speaker_led_l, LOW);
    Serial.println("TEST CHAMBER LEDs > COMPLETE");
    delay(500);

    // TEST TONE GENERATION
    Serial.println("TEST TONE GENERATION");
    delay(500);
    SPEAKER_RIGHT.play(CS_FREQUENCY);
    SPEAKER_LEFT.play(CS_FREQUENCY);
    delay(10000);
    SPEAKER_RIGHT.stop();
    SPEAKER_LEFT.stop();
    Serial.println("TEST TONE GENERATION > COMPLETE");
    delay(500);

    // TEST SHOCK GENERATION LEFT, THEN RIGHT
    Serial.println("TEST SHOCKER");
    delay(500);
    Serial.println("RIGHT SIDE");
    digitalWrite(shocker_r_pin, HIGH);
    delay(5000);
    digitalWrite(shocker_r_pin, LOW);
    Serial.println("LEFT SIDE");
    digitalWrite(shocker_l_pin, HIGH);
    delay(5000);
    digitalWrite(shocker_l_pin, LOW);
    Serial.println("TEST SHOCKER > COMPLETE");
    delay(500);

    // TEST OPTOGENETICS LED
    Serial.println("TEST OPTOGENETICS LED");
    digitalWrite(opto_LED, HIGH);
    delay(5000);
    digitalWrite(opto_LED, LOW);
    Serial.println("TEST OPTOGENETICS > COMPLETE");

    // TEST DORICS DIGITAL INPUTS
    Serial.println("TEST DORICS");
    delay(500);
    Serial.println("CHANNEL 2");
    digitalWrite(fiber_clock, HIGH);
    delay(5000);
    digitalWrite(fiber_clock, LOW);
    Serial.println("CHANNEL 3");
    digitalWrite(fiber_trial, HIGH);
    delay(5000);
    digitalWrite(fiber_trial, LOW);
    Serial.println("TEST DORICS > COMPLETE");
    delay(500);


  }

  // RESET CUMMULATIVE VARIABLE VALUES
  TOTAL_AVOIDANCE_FAILURE = 0;
  TOTAL_AVOIDANCE_SUCCESS = 0;
  ESCAPE_LATENCY_CUMULATIVE = 0;

  // START SESSION
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

    // INITIATE FIBER PHOTOMETRY CLOCK (DATA COLLECTION START)
    Serial.println("FIBER CLOCK > START");
    digitalWrite(fiber_clock, HIGH);

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
      // Trial number marks the beginning of fiber trial timekeeping
      digitalWrite(fiber_trial, HIGH);

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
            digitalWrite(shocker_r_pin, HIGH);
            Serial.println("US > ON");

            // KEEP US FOR SPECIFIC TIME DELAY
            for (int i = 0; i < SHOCK_DURATION; i++) {
              delay(1000);
            }

            // TERMINATE SHOCKER
            digitalWrite(shocker_l_pin, LOW);
            digitalWrite(shocker_r_pin, LOW);
            Serial.println("US > OFF");

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
        // Trial number marks end of fiber trial timekeeping
        digitalWrite(fiber_trial, LOW);

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

      // BEGIN MOTION DETECTION TIMER
      MOTION_DETECTION_START = millis();

      // DETECT POSITION, DELIVER CS AND US
      while (true) {
        L1_READING = IR_SENSOR_L1.distance(); sensor_cal.CheckDelay(); L1_READING = IR_SENSOR_L1.distance();
        L2_READING = IR_SENSOR_L2.distance(); sensor_cal.CheckDelay(); L2_READING = IR_SENSOR_L2.distance();
        R1_READING = IR_SENSOR_R1.distance(); sensor_cal.CheckDelay(); R1_READING = IR_SENSOR_R1.distance();
        R2_READING = IR_SENSOR_R2.distance(); sensor_cal.CheckDelay(); R2_READING = IR_SENSOR_R2.distance();

        if (R1_READING < IF_THRESHOLD
            || R2_READING < IF_THRESHOLD) {
          RIGHT_ACTIVE = HIGH;
          LEFT_ACTIVE = LOW;
        } else if (L1_READING < IF_THRESHOLD
                   || L2_READING < IF_THRESHOLD) {
          LEFT_ACTIVE = HIGH;
          RIGHT_ACTIVE = LOW;
        } else {
          RIGHT_ACTIVE = LOW;
          LEFT_ACTIVE = LOW;

          // END MOTION DETECTION TIMER
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
          digitalWrite(speaker_led_r, HIGH);
          digitalWrite(speaker_led_l, HIGH);

          // BEGIN TIMER FOR TONE DURATION (14 sec, 1 sec shock)
          START_TONE = millis();
          CURRENT_TONE_DELAY = millis();

          // BEGIN TIMER FOR OPTOGENETICS LED
          OPTO_START = millis();
          OPTO_CURRENT = millis();
          int TRIAL_CHECK = 0;

          // RESETTING OPTOGENETICS TIMESTAMP VARIABLES
          OPTO_START_TIMESTAMP = 0;
          OPTO_END_TIMESTAMP = 0;

          // ADD 0.5 SEC DELAY TO AVOID SENSOR DETECTION ARTIFACTS
          unsigned long S_DELAY_START = millis();
          unsigned long S_DELAY_CURRENT = millis();
          while (S_DELAY_START - S_DELAY_CURRENT < 500) {
            S_DELAY_CURRENT = millis();
          }

          // WHILE THE OPPOSITE COMPARTMENT IS NOT ACTIVE, CONTINUE FOR TONE_DURATION
          while (true) {

            // OPTOGENETICS PROTOCOL. Since OPTO_FLICKER_DURATION is written in Hz, we have to convert it to ms. We divide the period
            // by 2 to reflect the actual blink duration.
            OPTO_CURRENT = millis();
            OPTO_FLICKER_START = millis();
            // TURN ON LED OPTOGENETICS FOR OPTO_FULL_DURATION
            if ((OPTO_CURRENT - OPTO_START) < (OPTO_FULL_DURATION * 1000)){
              if (OPTO_FLICKER_START - OPTO_FLICKER_PREVIOUS >= ((1/OPTO_FLICKER_DURATION)*500)){
                OPTO_FLICKER_PREVIOUS = OPTO_FLICKER_START;

                if (OPTO_STATE == LOW){
                  OPTO_STATE = HIGH;
                } else {
                  OPTO_STATE = LOW;
                }

                digitalWrite(opto_LED, OPTO_STATE);
              }
              if (OPTO_START_TIMESTAMP == 0){
                Serial.println("OPTO > ON");
                OPTO_START_TIMESTAMP += 1;
              }
            }else if (OPTO_END_TIMESTAMP == 0){
              Serial.println("OPTO > OFF");
              digitalWrite(opto_LED, LOW);
              OPTO_END_TIMESTAMP += 1;
              break;
            }

            L1_READING = IR_SENSOR_L1.distance(); sensor_cal.CheckDelay(); L1_READING = IR_SENSOR_L1.distance();
            L2_READING = IR_SENSOR_L2.distance(); sensor_cal.CheckDelay(); L2_READING = IR_SENSOR_L2.distance();

            // CHECK IF LEFT IS ACTIVE
            if (L1_READING < IF_THRESHOLD
                || L2_READING < IF_THRESHOLD) {
              LEFT_ACTIVE = HIGH;
              RIGHT_ACTIVE = LOW;
            } else {
              LEFT_ACTIVE = LOW;
              RIGHT_ACTIVE = LOW;
            }

            CURRENT_TONE_DELAY = millis();

            if ((LEFT_ACTIVE == HIGH) && (TRIAL_CHECK==0)){

              // ENSURE OPTO LED IS TURNED ON FOR ONE SEC
              float OPTO_NOW = millis();
              digitalWrite(opto_LED, HIGH);

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

              OPTO_CURRENT = millis();
              while ((OPTO_CURRENT - OPTO_NOW) <= 1000){
                OPTO_CURRENT = millis();
              }

              TRIAL_CHECK = 1;
            }

            // AFTER SPECIFIC DELAY, TRIGGER US
            if (((CURRENT_TONE_DELAY - START_TONE) > (DELTA_TONE_SHOCK * 1000)) && (TRIAL_CHECK==0)) {

              // ENSURE OPTO LED IS TURNED ON
              digitalWrite(opto_LED, HIGH);

              // TRIGGER US
              digitalWrite(shocker_l_pin, HIGH);
              digitalWrite(shocker_r_pin, HIGH);
              Serial.println("US > ON");

              // KEEP US FOR SPECIFIC TIME DELAY
              for (int i = 0; i < SHOCK_DURATION; i++) {
                delay(1000);
              }

              // TERMINATE SHOCK
              digitalWrite(shocker_l_pin, LOW);
              digitalWrite(shocker_r_pin, LOW);
              Serial.println("US > OFF");

              // TERMINATE TONE IN THE RIGHT COMPARTMENT IF AFTER SHOCK
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // TERMINATE OPTOGENETICS LED AFTER 15 SEC
              digitalWrite(opto_LED, LOW);
              if (OPTO_END_TIMESTAMP == 0){
                Serial.println("OPTO > OFF");
                OPTO_END_TIMESTAMP += 1;
              }

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
          Serial.println("CS > ON");

          // RECORD LATENCY_START
          ESCAPE_LATENCY_START = millis();

          // TURN LED ON IN BOTH SIDES
          digitalWrite(speaker_led_r, HIGH);
          digitalWrite(speaker_led_l, HIGH);

          // BEGIN TIMER FOR TONE DURATION (14 sec, 1 sec shock)
          START_TONE = millis();
          CURRENT_TONE_DELAY = millis();

          // BEGIN TIMER FOR OPTOGENETICS LED
          OPTO_START = millis();
          OPTO_CURRENT = millis();

          // RESETTING OPTOGENETICS TIMESTAMP VARIABLES
          OPTO_START_TIMESTAMP = 0;
          OPTO_END_TIMESTAMP = 0;
          int TRIAL_CHECK = 0;

          // ADD 0.5 SEC DELAY TO AVOID SENSOR DETECTION ARTIFACTS
          unsigned long S_DELAY_START = millis();
          unsigned long S_DELAY_CURRENT = millis();
          while (S_DELAY_START - S_DELAY_CURRENT < 500) {
            S_DELAY_CURRENT = millis();
          }

          // WHILE THE OPPOSITE COMPARTMENT IS NOT ACTIVE, CONTINUE FOR TONE_DURATION
          while (true) {

            // OPTOGENETICS PROTOCOL
            OPTO_CURRENT = millis();
            OPTO_FLICKER_START = millis();
            // TURN ON LED OPTOGENETICS FOR OPTO_FULL_DURATION
            if ((OPTO_CURRENT - OPTO_START) < (OPTO_FULL_DURATION * 1000)){
              if (OPTO_FLICKER_START - OPTO_FLICKER_PREVIOUS >= ((1/OPTO_FLICKER_DURATION)*500)){
                OPTO_FLICKER_PREVIOUS = OPTO_FLICKER_START;

                if (OPTO_STATE == LOW){
                  OPTO_STATE = HIGH;
                } else {
                  OPTO_STATE = LOW;
                }

                digitalWrite(opto_LED, OPTO_STATE);
              }
              if (OPTO_START_TIMESTAMP == 0){
                Serial.println("OPTO > ON");
                OPTO_START_TIMESTAMP += 1;
              }
            }else if (OPTO_END_TIMESTAMP == 0){
              Serial.println("OPTO > OFF");
              digitalWrite(opto_LED, LOW);
              OPTO_END_TIMESTAMP += 1;
              break;
            }

            R1_READING = IR_SENSOR_R1.distance(); sensor_cal.CheckDelay(); R1_READING = IR_SENSOR_R1.distance();
            R2_READING = IR_SENSOR_R2.distance(); sensor_cal.CheckDelay(); R2_READING = IR_SENSOR_R2.distance();

            // CHECK IF RIGHT IS ACTIVE
            if (R1_READING < IF_THRESHOLD
                || R2_READING < IF_THRESHOLD) {
              RIGHT_ACTIVE = HIGH;
              LEFT_ACTIVE = LOW;
            } else {
              RIGHT_ACTIVE = LOW;
              LEFT_ACTIVE = LOW;
            }

            if ((RIGHT_ACTIVE == HIGH) && (TRIAL_CHECK == 0)) {

              // ENSURE OPTO LED IS TURNED ON FOR ONE SEC
              float OPTO_NOW = millis();
              digitalWrite(opto_LED, HIGH);

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

              OPTO_CURRENT = millis();
              while ((OPTO_CURRENT - OPTO_NOW) <= 1000){
                OPTO_CURRENT = millis();
              }

              TRIAL_CHECK = 1;
            }

            // AFTER SPECIFIC DELAY, TRIGGER US
            if (((CURRENT_TONE_DELAY - START_TONE) > (DELTA_TONE_SHOCK * 1000)) && (TRIAL_CHECK == 0)) {

              // ENSURE OPTO LED IS TURNED ON
              digitalWrite(opto_LED, HIGH);

              // TRIGGER US
              digitalWrite(shocker_l_pin, HIGH);
              digitalWrite(shocker_r_pin, HIGH);
              Serial.println("US > ON");

              // KEEP US FOR SPECIFIC TIME DELAY
              for (int i = 0; i < SHOCK_DURATION; i++) {
                delay(1000);
              }

              // TERMINATE SHOCKER
              digitalWrite(shocker_l_pin, LOW);
              digitalWrite(shocker_r_pin, LOW);
              Serial.println("US > OFF");

              // TERMINATE TONE IN THE COMPARTMENT IF AFTER SHOCK
              SPEAKER_RIGHT.stop();
              SPEAKER_LEFT.stop();
              Serial.println("CS > OFF");

              // TERMINATE OPTOGENETICS LED AFTER 15 SEC
              digitalWrite(opto_LED, LOW);
              if (OPTO_END_TIMESTAMP == 0){
                Serial.println("OPTO > OFF");
                OPTO_END_TIMESTAMP += 1;
              }

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
            digitalWrite(shocker_r_pin, HIGH);
            Serial.println("US > ON");

            // KEEP US FOR SPECIFIC TIME DELAY
            for (int i = 0; i < SHOCK_DURATION; i++) {
              delay(1000);
            }

            // TERMINATE SHOCKER
            digitalWrite(shocker_l_pin, LOW);
            digitalWrite(shocker_r_pin, LOW);
            Serial.println("US > OFF");

            // RECORD LATENCY_END WHEN NO SHUTTLING
            ESCAPE_LATENCY_END = 0;

            break;
        }

      }

      // SELECT RANDOM ITI FROM LIST
      ITI_DURATION = ITI_INTERVALS[random(0, 5)];
      Serial.print("INTER-TRIAL-INTERVAL (SEC): "); Serial.println(ITI_DURATION);

      // INITIATE INTER-TRIAL-INTERVAL
      delay(ITI_DURATION*1000);

      // Trial number marks end of fiber trial timekeeping
      digitalWrite(fiber_trial, LOW);


      // PRINT EXIT INFORMATION ABOUT TRIAL
      // ###########################################################################
      Serial.print("TRIAL "); Serial.print(x+1); Serial.println(" > END");
      Serial.print("TRIAL "); Serial.print(x+1); Serial.print(" ESCAPE LATENCY (SEC): "); Serial.println((ESCAPE_LATENCY_END - ESCAPE_LATENCY_START)/1000.0);
      Serial.print("CUMULATIVE TOTAL SHUTTLINGS: "); Serial.println(TOTAL_AVOIDANCE_SUCCESS);
      Serial.print("CUMULATIVE TOTAL FAILURES: "); Serial.println(TOTAL_AVOIDANCE_FAILURE);

    }

    // TERMINATE FIBER PHOTOMETRY CLOCK (DATA COLLECTION END)
    Serial.println("FIBER CLOCK > END");
    digitalWrite(fiber_clock, LOW);

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
