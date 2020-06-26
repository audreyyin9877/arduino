
/* --------------------------------------------------------------------------------
 * LeDoux Lab 2020
 * 
 * Jose Oliveira da Cruz 
 * jose.cruz@nyu.edu
 * 
 * --------------------------------------------------------------------------------
 */

// Constants

const int acclimation_seconds = 5 * 60L; // IN SECONDS
const int cooldown_seconds = 5 * 60L;    // IN SECONDS

const int cs_len = 30;              // DURATION CS

// Variables

int total_cs_number = 5;          // NUMBER OF CS 

int switchstate = 0; // Button starts the experiment
int switchstate_test_led = 0; // Test the led
int detect_cs = 0; // detect if the cs was delivered.
int detect_us = 0; // detect if the us was delivered

// list of the inter-trial-intervals: ITI
int itintervals[] = {60, 90, 120, 160, 180};


void setup() {
  // Start serial
  Serial.begin(9600); // serial will be read by bonsai
  delay(5 * 1000);
  Serial.println("LEDOUX LAB");
  
  // Digital Output pins
  pinMode(7, OUTPUT); // Delivering the CS
  pinMode(8, OUTPUT); // Delivering the US
  pinMode(9, OUTPUT); // Delivering the CHAMBER LED when the CS is delivered.
  pinMode(11, OUTPUT); // Signal the deliver of CS to LED
  pinMode(12, OUTPUT); // Signal the deliver of US to LED

  // Button to start the experiment.
  pinMode(2, INPUT); // single puch to start
  pinMode(3, INPUT); // button to test LEDs
  
}

void loop() {

  // TURN CHAMBER LED ON
  switchstate_test_led = digitalRead(3);

  if (switchstate_test_led == HIGH) {
    Serial.println("CHAMBER LED: ON");
    digitalWrite(9, HIGH);
    
    delay(5000);
    
    digitalWrite(9, LOW);
    Serial.println("CHAMBER LED: OFF");
  }
  
  //int total_cs_number = 5; // How many CS is going to be delivered.
  
  // variable to keep track of the number of CS at a give time
  int current_cs = 1;


  // START NEW EXPERIMENT
  switchstate = digitalRead(2);
  if (switchstate == HIGH) { 

    Serial.println("NEW EXPERIMENT: CLASSICAL THREAT CONDITIONING");
    delay(1000);
    Serial.println("SESSION: TESTING"); // SESSION NAME
    delay(1000);
    
    // TEST CHAMBER LEDs
    Serial.println("TEST CHAMBER LED");
    Serial.println("LED: ON");
    digitalWrite(9, HIGH);
    delay(10*1000); // 10 seconds
    digitalWrite(9, LOW);
    Serial.println("LED: OFF");
    delay(5 * 1000);

    //  PRINT INFORMATION ABOUT SESSION

    Serial.print("NUMBER OF CSs: ");
    Serial.println(total_cs_number);
    delay(500);

    Serial.print("CS DURATION (sec): ");
    Serial.println(cs_len);
    delay(500);
    

    // START THE EXPERIMENT
    
    Serial.println("NEW EXPERIMENT: START");
    delay(1000);

    Serial.print("ACCLIMATION (sec): ");
    Serial.println(acclimation_seconds);
    delay(acclimation_seconds * 1000L);
    
    while (total_cs_number > 0) {

      Serial.print("CS: 0");
      Serial.println(current_cs);
      
      // CS --> ON
      digitalWrite(7, HIGH);
      digitalWrite(9, HIGH); // CHAMBER LED: ON
      digitalWrite(11, HIGH); // ARDUINO LED: ON 
      
      // CS DURATION
      delay( (cs_len) * 1000L);
      
      // CS --> OFF
      digitalWrite(7, LOW);
      digitalWrite(9, LOW); // CHAMBER LED: OFF
      digitalWrite(11, LOW); // ARDUINO LED: OFF
      
      digitalWrite(9, LOW);
    
      // WAIT ONE SECOND
      delay(1000);

      // CALCULATE REMAINING CS
      total_cs_number = total_cs_number - 1;
      
      // CURRENT CS NUMBER
      current_cs = current_cs + 1;

      // PRINT INTER-TRIAL-INTERVAL 
      int delay_iti = itintervals[random(0, 5)];
      
      Serial.print("INTER-TRIAL-INTERVAL (sec): ");
      Serial.println(delay_iti);
      
      delay(delay_iti * 1000L); // Transform the delay into seconds
      
    }

  // COOLDOWN AFTER EXPERIMENT CYCLE
  Serial.print("COOLDOWN (sec): ");
  Serial.println(cooldown_seconds);
  delay(cooldown_seconds * 1000L);

 
  Serial.println("NEW EXPERIMENT: END");
  }
}
