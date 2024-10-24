/* 
   Controller board: Arduino Uno R3
   
   Motor drivers: CL57T       (StepperOnline)
   Motors: 17HS08-1004D-E1K   (StepperOnline)

   Libraries: AccelStep.h - ver: 1.64.0 by Patrick Wasp 

   Lasr update: Oct 24, 2024

   Sources of information:
      https://www.youtube.com/watch?v=vCaB5zJacrU
      https://github.com/jumejume1/stepper-encoder-close-loop-test/blob/main/test_stepper_encoder.ino

      https://www.youtube.com/watch?v=bsB5shP3vls ; 
      https://www.youtube.com/watch?v=dTGITLnYAY0 ; https://github.com/curiores/ArduinoTutorials/tree/main ; https://curiores.com/positioncontrol
      https://all3dp.com/2/grbl-software-guide/ ; 
      https://help.stepperonline.com/en/article/wiring-diagram-for-closed-loop-stepper-motor-4cddqy/ ; 
      https://www.instructables.com/Control-Nema-Stepper-Motor-With-Arduino-and-Micro-/ ; 
   
   Notes:
     1. EA- and EB- are not connected, they are common with EGND
     2. Vcc is connected to Arduino Uno R3 5V+
     3. EGND is connected to Arduino Uno R3 GND  (see screenshot picture)
*/

#include <AccelStepper.h>
#include <Wire.h>
#include <Waveshare_LCD1602_RGB.h>

Waveshare_LCD1602_RGB rgb_lcd(16,2);  // 16 characters and 2 lines of text
 
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

char lcdText_1[17] = "";
char lcdText_2[17] = "";

////// STEPPER MOTORS DIRECTION AND STEP PINS:
const byte dirPin_1 = 2;      // DIR+ pin, encoder #1
const byte stepPin_1 = 5;     // PUL+ pin, encoder #1
const byte enablePin_1 = 8;   // ENA+ pin, encoder #1

const byte dirPin_2 = 3;      // DIR+ pin, encoder #2
const byte stepPin_2 = 6;     // PUL+ pin, encoder #2
const byte enablePin_2 = 9;   // ENA+ pin, encoder #2

// Define motor interface type (type 1 means an external stepper driver with Step and Direction pins)
#define motorInterfaceType 1

// Define steppers and the pins they use
AccelStepper stepper_1(motorInterfaceType, stepPin_1, dirPin_1);
AccelStepper stepper_2(motorInterfaceType, stepPin_2, dirPin_2);

// Global variables
volatile int encoder_1_count = 0;
volatile int encoder_2_count = 0;

int distance_1 = 2000;
int distance_2 = 2000;

int max_speed_1 = 4000; 
int max_speed_2 = 4000;

int acceleration_1 = 3000;
int acceleration_2 = 3000;

char str_1[] = "0";
char s_text_1[65] = "_";

char str_2[] = "0";
char s_text_2[65] = "_";

unsigned long previousTimeLed_1 = millis();
unsigned long timeIntervalLed_1 = 100;         // in milliseconds

unsigned long nloops = 0;         // count # of loop() iterations

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1); 
  Serial.println("-------------------------------------");
  Serial.println("Device is initialized. Status: Ready.");

  pinMode(enablePin_1, OUTPUT);     // motor driver #1 - enable pin
  pinMode(enablePin_2, OUTPUT);     // motor driver #2 - enable pin
  delay(100);
  digitalWrite(enablePin_1, LOW);   // enable motor driver #1
  digitalWrite(enablePin_2, LOW);   // enable motor driver #2
  
  // encoder_1 PUL+ pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(stepPin_1), readEncoder_1, RISING);
  
  // encoder_2 PUL+ pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(stepPin_2), readEncoder_2, RISING);

  // initialize Waveshare RGB LCD
  rgb_lcd.init();
  rgb_lcd.setCursor(0, 0);
  rgb_lcd.print("Nema 17 Stepper");
  rgb_lcd.setCursor(0, 1);
  rgb_lcd.print("17HS08-1004D-E1K");
  rgb_lcd.BlinkLED();
  delay(3000);
  rgb_lcd.noBlinkLED();

  //// STEPPER PROPERTIES:
  /* From AccelStepper: "Max possible speed depends on clock speed. Caution: Speeds that exceed the maximum 
     speed supported by the processor may Result in non-linear accelerations and decelerations" */
  stepper_1.setMaxSpeed(max_speed_1);          // 4000
  stepper_1.setAcceleration(acceleration_1);   // 4000
  stepper_2.setMaxSpeed(max_speed_1);          // 4000
  stepper_2.setAcceleration(acceleration_2);   // 4000

  Serial.println("-------------------------------------");
  
  Serial.print("distance_1 = ");
  Serial.println(distance_1);
  Serial.print("distance_2 = ");
  Serial.println(distance_2);
  
  Serial.print("max_speed_1 = ");
  Serial.println(max_speed_1);
  Serial.print("max_speed_2 = ");
  Serial.println(max_speed_2);

  Serial.print("acceleration_1 = ");
  Serial.println(acceleration_1);
  Serial.print("acceleration_2 = ");
  Serial.println(acceleration_2);
  
  Serial.println("-------------------------------------");
   
  stepper_1.moveTo(distance_1);
  stepper_2.moveTo(distance_2);
  delay(1000);
  
  digitalWrite(enablePin_1, HIGH);   // disable motor driver #1
  digitalWrite(enablePin_2, HIGH);   // disable motor driver #2
  delay(3000);
  
  rgb_lcd.clear();
  
  itoa(encoder_1_count, str_1, 10);
  strcpy(lcdText_1, "Motor_1: ");
  strcat(lcdText_1, str_1); 
  rgb_lcd.setCursor(0, 0);
  rgb_lcd.print(lcdText_1);
  
  itoa(encoder_2_count, str_2, 10);
  strcpy(lcdText_2, "Motor_2: ");
  strcat(lcdText_2, str_2); 
  rgb_lcd.setCursor(0, 1);
  rgb_lcd.print(lcdText_2);

}  // Setup End

void loop() {

  nloops++;
  
  Serial.print("Started loop()... nloops = ");
  Serial.println(nloops);
  
  digitalWrite(enablePin_1, LOW);  // +FM: enables motor driver CL57T #1
  digitalWrite(enablePin_2, LOW);  // +FM: enables motor driver CL57T #2
  
  if( stepper_1.distanceToGo() == 0 ){
          digitalWrite(enablePin_2, LOW);   // +FM: enables motor driver CL57T #1
          distance_1 = distance_1 * -1;
          stepper_1.moveTo(distance_1 * -1);
          digitalWrite(enablePin_1, HIGH);  // +FM: disables motor driver CL57T #1
          delay(5000);
  }
  stepper_1.run();

  if( stepper_2.distanceToGo() == 0 ){
          digitalWrite(enablePin_2, LOW);   // +FM: enables motor driver CL57T #2
          distance_2 = distance_2 * -1;
          stepper_2.moveTo(distance_2 * -1);
          digitalWrite(enablePin_2, HIGH);  // +FM: disables motor driver CL57T #2
          delay(5000);
  }
  stepper_2.run();
  
  unsigned long currentTime = millis();
  
  // task 1
  if(currentTime - previousTimeLed_1 > timeIntervalLed_1) {
    previousTimeLed_1 = currentTime;

    Serial.println("-------------------------------------");
       
    itoa(encoder_1_count, str_1, 10);
    rgb_lcd.setCursor(9, 0);
    rgb_lcd.print(str_1);
    
    strcpy(s_text_1, "DEBUG_201: s_text_1: ");
    strcat(s_text_1, str_1);
    Serial.println(s_text_1);
    
    itoa(encoder_2_count, str_2, 10);
    rgb_lcd.setCursor(9, 1);
    rgb_lcd.print(str_2);
    
    strcpy(s_text_2, "DEBUG_209: s_text_2: ");
    strcat(s_text_2, str_2);
    Serial.println(s_text_2);

    Serial.println("-------------------------------------");
//    Serial.print("Ending loop()... nloops = ");
//    Serial.println(nloops);
  }
}   // end of loop()

void readEncoder_1() {   // Rising edge of wave A initiates pin B state reading to determine direction

  // If pin B is low when A trips, then A precedes B
  if(digitalRead(dirPin_1) == LOW) {
    encoder_1_count = encoder_1_count + 1; //Positive Rotation
  }

  if(digitalRead(dirPin_1) == HIGH) {
    encoder_1_count = encoder_1_count - 1; // Negative Rotation
  }
  
  Serial.print("readEncoder_1(): encoder_1 count: ");
  Serial.println(encoder_1_count);
  
  //rgb_lcd.clear();
  //rgb_lcd.print(str_1);
  delay(10);
  // use for debugging - remember to comment out
}

void readEncoder_2() {   // Rising edge of wave A initiates pin B state reading to determine direction

  // If pin B is low when A trips, then A precedes B
  if(digitalRead(dirPin_2) == LOW) {
    encoder_2_count = encoder_2_count + 1; //Positive Rotation
  }

  if(digitalRead(dirPin_2) == HIGH) {
    encoder_2_count = encoder_2_count - 1; // Negative Rotation
  }
  
  Serial.print("readEncoder_2(): encoder_2 count: ");
  Serial.println(encoder_2_count);
  
  //rgb_lcd.clear();
  //rgb_lcd.print(str_2);
  delay(10);
  // use for debugging - remember to comment out
}
