// Definitions
  // [ Servos ]
    #include <Servo.h>
    #define BottleHugger 4 // To keep bottle in place after IR == LOW (closest to reset button on RAMPS 1.4 board)
    Servo servo1; //make object for servo1 (bottle hugger)

  // [ Define the pins for motor 1 (psi motor) ]
    #define stepPin1 26 // Pin 26 for steps of psi motor1
    #define dirPin1 28 // Pin 28 for direction of psi motor1
    #define enable1 24 // Pin 24 for motor 1 enable

  // [ Define the pins for motor 2 (cap screw motor)] 
    #define stepPin2 36 // Pin 36 for steps of cap screw motor2 (E1)
    #define dirPin2 34 // Pin 34 for direction of cap screw motor2 (E1)
    #define enable2 30 // Pin 30 for motor 2 enable (E1)

  // [ Define the pins for motor 3 (heat motor)]
    #define stepPin3 54 // Pin 54 for steps of heat motor3
    #define dirPin3 55 // Pin 55 for direction of heat motor3
    #define enable3 38 // Pin 38 for motor 3 enable

  // Define the pin for the RodSwitch (Xmin endstop)
    #define RodSwitch 3 // Pin 3, Xmin sensor
  // Define the pin for the BottleIR (Zmax sensor)
    #define BottleIR 19 // Pin 19, Zmax sensor
  // Define the pin for the Zmin endstop
    #define heatgunSwitch 18 // Pin 18, Zmin sensor

  // [ Stepper counts & states ]
    volatile int Motor1State = 0;
    volatile int stepCount1 = 0;
    const int stepsPerRev1 = 4758; // Rod motor
    volatile bool screwBottleOn = false; //flag to permit CW turning of motor 1(screwing bottle on)
    volatile bool turnBottleFlag = false; //flag to turn bottle constantly or not
    volatile int isrCounter = 0;// Declare a counter variable outside the ISR
    volatile int Motor2State = 1;
    volatile int stepCount2 = 0;
    const int stepsPerRev2 = 4022; // Cap Screw Motor
    volatile bool heatgunFlag = false; //flag to allow heatgun state 1 to commence
    volatile int Motor3State = 0;
    volatile int stepCount3CW = 0;
    volatile int stepCount3CCW = 0;
    volatile int dirPin3state = 1;

    const int stepsPerRev3 = 5101; // Heatgun motor

  // [ Pressure Sensor ] ///////////////////////////////////
  #define airPump 10 // Setting the dc motor for the air compressor on D10
  #include <Wire.h> 
  #include <SparkFun_MicroPressure.h>
  SparkFun_MicroPressure mpr; // sensor type
  float pressure = 0;
  volatile bool airCompressorFlag = false;
  bool sensorDetected = false;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastSensorRead = 0;

    float readP = 0.0;
    unsigned long timePrevP = 0; // The previous time for elapsed time calculation
    unsigned long currentMillisP = 0;
    unsigned long previousMillisP = 0; // For interval timing
    const long intervalP = 100; // PID update interval in milliseconds
    float PPID_error = 0;float Pprevious_error = 0;float PPID_value = 0;
    float elapsedTimeP, TimeP;
    float last_setP = 0;
    float setP = 25;
  ///////////////////////////////////////////////////////
  // [ Psi PID CONSTANTS ]
    //////////////////////////////////////////////////////////
    float kpP = 17.69;   float kiP = 1.41;   float kdP = 55.3;
    //////////////////////////////////////////////////////////
    int PPID_p = 0;    int PPID_i = 0;    int PPID_d = 0;
    float last_kpP = 0; float last_kiP = 0; float last_kdP = 0;

  // [ Fans ]
  #define heatgunFan 8 //Heatgun fan on D8

  // [ Heating Element ] ///////////////////////////////////////
  #include <math.h>
  #define heatingElement 9 //Heating Element on D9
  #define Thermistor1 A13 // T0 on RAMPS 1.4 corresponds to A13 on the Arduino Mega
  #define WINDOW_SIZE 5 // sample size for moving average
  float temperatureBuffer[WINDOW_SIZE] = {0}; //Buffer for moving average
  int bufferIndex = 0; 
  const float constA = 1.26570827e-03;
  const float constB = 1.57485712e-04;
  const float constC = 1.80331120e-07;
  float temperature = 0;
    float setT = 0;
    float readT = 0.0;
    unsigned long timePrevT = 0; // The previous time for elapsed time calculation
    unsigned long currentMillisT = 0;
    unsigned long previousMillisT = 0; // For interval timing
    const long intervalT = 100; // PID update interval in milliseconds
    float TPID_error = 0;float Tprevious_error = 0;float TPID_value = 0;
    float MPID_error = 0;float Mprevious_error = 0;float MPID_value = 0;
    float elapsedTimeT, TimeT;
    float last_setT = 0;
  ///////////////////////////////////////////////////////
  // [ Temp PID CONSTANTS ]
    //////////////////////////////////////////////////////////
    float kpT = 70.69;   float kiT = 1.41;   float kdT = 5.3;
    //////////////////////////////////////////////////////////
    int TPID_p = 0;    int TPID_i = 0;    int TPID_d = 0;
    float last_kpT = 0; float last_kiT = 0; float last_kdT = 0;
  // [ LCD Bizz ] ///////////////////////////////////////
    #include <LiquidCrystal_I2C.h>
    int LCDclear = 0;
    LiquidCrystal_I2C lcd(0x27,20,4);  //0x3f or 0x27 
  
    //Custom Characters:
    byte subscript_p[8] = {0b00000,0b00000,0b00000,0b00110,0b01001,0b01001,0b01110,0b01000};
    byte subscript_i[8] = {B00000,B00000,B00000,B01000,B00000,B01000,B01000,B01000};
    byte subscript_d[8] = {B00000,B00000,B00010,B00010,B01110,B10010,B10010,B01100};
    byte therm_med[8] = {B01110,B01010,B01010,B01010,B01110,B11111,B11111,B01110};
    byte therm_cold[8] = {B01110,B01010,B01010,B01010,B01010,B10001,B11111,B01110};
    byte therm_hot[8] = {B01110,B01010,B01110,B01110,B01110,B11111,B11111,B01110};
    byte motor[8] = {B00000,B01110, B01110, B00100,B11111,B10001,B10001,B11111};
    byte heatgun[8] = {B00011,B11101,B11101,B11011,B00000,B01000,B10000,B00000};
    byte psi_p[8] = {B00000,B00000,B11100,B10010,B10010,B11100,B10000,B10000};
    byte psi_si[8] = {B00000,B00000,B00000,B11101,B10000,B01001,B00101,B11101};
    // Encoder //
      volatile bool button_pressed = false;
      volatile int menu_state = 0;
      volatile bool clk_State = false;
      volatile bool dt_State = false;
      volatile bool Last_State = false;
      unsigned long buttonPressTime = 0;  // Variable to store the time when the button is pressed

      // #define clk 49;      //Pin 1 from rotary encoder
      // #define data 51;     //Pin 2 from rotary encoder
      // //#define encoderSwitch 53; //SW pin for encoder

void setup() {
  Serial.begin(9600);
  Serial.println("MEGA START!");

  EncoderSetup();
  LCDSetUp();
  RodStepperSetUp();
  HeatStepperSetup();
  CapScrewStepperSetup();
  ServoSetup();
  SensorSetup();
  HeatingElementSetUp();
  FansSetup();
}
void EncoderSetup(){
  // Set the pin modes for the encoder
  pinMode(49, INPUT); // CLK
  pinMode(51, INPUT); // DT
  pinMode(53, INPUT); // Button

  // Enable pin change interrupt for PCINT0, PCINT1, PCINT2, and PCINT3 (pins 53 to 50)
  PCICR |= (1 << PCIE0); // Enable PCINT0 interrupt
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3); // Enable interrupts for pins 53 to 50

  // Enable pin change interrupt for PCINT16 to PCINT23 (pins 49 to 42)
  PCICR |= (1 << PCIE2); // Enable PCINT2 interrupt
  PCMSK2 |= (1 << PCINT16); // Enable interrupt for pin 49 (PCINT16)
}
void LCDSetUp(){
  lcd.init(); lcd.backlight(); 
  lcd.createChar(0, subscript_p);lcd.createChar(1, subscript_i);lcd.createChar(2, subscript_d);
  lcd.createChar(3, therm_med);lcd.createChar(4, therm_cold);lcd.createChar(5,therm_hot);lcd.createChar(6,motor);
  lcd.createChar(7, heatgun);lcd.createChar(8, psi_p); lcd.createChar(9, psi_si);

  lcd.print("Initializing...");
}
// Stepper Setup function to initialize motor 1
void RodStepperSetUp() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enable1, OUTPUT);

  cli(); // Disable global interrupts
  TCCR1A = 0; // Clear Timer/Counter Control Registers
  TCCR1B = 0;
  TCNT1 = 0; // Initialize counter value to 0

  // Set Compare Match Register for desired step rate
  OCR1A = 600; // fast frequency

  // Turn on CTC mode
  TCCR1B |= (1 << WGM12);

  // Set prescaler to 8 and start the timer
  TCCR1B |= (1 << CS11);

  // Enable Timer Compare Interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // Enable global interrupts

  // Set initial direction to CCW (towards endstop)
  digitalWrite(dirPin1, LOW);
  Serial.println("Rod motor setup complete");
}
// Stepper Setup function to initialize motor 2
void CapScrewStepperSetup() {
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enable2, OUTPUT);
  pinMode(airPump, OUTPUT); 
  digitalWrite(airPump, LOW);


  // Timer4 setup
  cli(); // Disable global interrupts
  TCCR4A = 0; // Clear Timer/Counter Control Registers
  TCCR4B = 0;
  TCNT4 = 0; // Initialize counter value to 0

  // Set Compare Match Register for desired step rate
  OCR4A = 2000; // high means slow

  // Turn on CTC mode
  TCCR4B |= (1 << WGM42);

  // Set prescaler to 8 and start the timer
  TCCR4B |= (1 << CS41);

  // Enable Timer Compare Interrupt
  TIMSK4 |= (1 << OCIE4A);
  sei(); // Enable global interrupts

  // Set initial direction to desired direction (CCW or CW)
  digitalWrite(dirPin2, LOW);
  Serial.println("Cap screw motor setup complete");
}
// Stepper Setup function to initialize motor 3
void HeatStepperSetup() {
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(enable3, OUTPUT);


  // Timer3 setup
  cli(); // Disable global interrupts
  TCCR3A = 0; // Clear Timer/Counter Control Registers
  TCCR3B = 0;
  TCNT3 = 0; // Initialize counter value to 0

  // Set Compare Match Register for desired step rate
  OCR3A = 3600; // low means fast

  // Turn on CTC mode
  TCCR3B |= (1 << WGM32);

  // Set prescaler to 8 and start the timer
  TCCR3B |= (1 << CS31);

  // Enable Timer Compare Interrupt
  TIMSK3 |= (1 << OCIE3A);
  sei(); // Enable global interrupts

  // Set initial direction to CCW (towards endstop)
  digitalWrite(dirPin3, LOW);
  Serial.println("Heat motor setup complete");
}
void HeatingElementSetUp() {
  
  setT = 200; //automatic setT
  Serial.println("Heating element setup complete");

}
void ServoSetup(){
  servo1.attach(BottleHugger); 
  Serial.println("Servo setup complete");
}
void SensorSetup() {
  // Set up the endstop switches
  pinMode(RodSwitch, INPUT_PULLUP);
  pinMode(BottleIR, INPUT_PULLUP);
  pinMode(heatgunSwitch, INPUT_PULLUP);

  // Pressure Sensor 
  Wire.begin();
  if (!mpr.begin()) {
    Serial.println("Sensor not detected. Please check wiring.");
    sensorDetected = false;
  } else {
    Serial.println("Sensor detected!");
    sensorDetected = true;
  }
}
void FansSetup(){
  pinMode(heatgunFan, OUTPUT);
  digitalWrite(heatgunFan, HIGH);
}
float tempCalc(float Thermistor_){
  
  // Using a voltage divider to calculate resistance from thermistor
  float adcValue = analogRead(Thermistor_);
  float voltage = adcValue * (5.0 / 1024.0); // Calculate the voltage at the thermistor
  float ResTherm = 4700 / (5.0 / voltage - 1.0); // Calculate the thermistor resistance

  // Steinhart-Hart Equation to convert resistance to temperature, from Kelvin to Celsius
  float logRes = log(ResTherm);
  float Temp = 1.0 / (constA + constB * logRes + constC * pow(logRes, 3)) - 273.15;

  return(Temp);
}
// Temperature Moving Average Logic to Remove Noisy Spikes
float tempAvg(float algTemp) {
    temperatureBuffer[bufferIndex] = algTemp;
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += temperatureBuffer[i];
    }
    sum = sum / WINDOW_SIZE;

    return sum;
}
void updateTempPID() {
    // PID calculations
    readT = tempCalc(Thermistor1);
    TPID_error = setT - readT;
    TPID_p = kpT * TPID_error;
    TPID_i += kiT * TPID_error * (intervalT / 1000.0);
    TPID_i = constrain(TPID_i, -20, 20);
    timePrevT = TimeT;
    TimeT = micros();
    elapsedTimeT = (TimeT - timePrevT) / 1000.0;
    TPID_d = kdT * ((TPID_error - Tprevious_error) / elapsedTimeT);
    TPID_value = TPID_p + TPID_i + TPID_d;
    TPID_value = constrain(TPID_value, 0, 255);
    analogWrite(heatingElement, TPID_value);
    Tprevious_error = TPID_error;
}
void updatePsiPID() {
    // PID calculations
    readP = mpr.readPressure();
    PPID_error = setP - readP;
    PPID_p = kpP * PPID_error;
    PPID_i += kiP * PPID_error * (intervalP / 1000.0);
    PPID_i = constrain(PPID_i, -20, 20);
    timePrevP = TimeP;
    TimeP = micros();
    elapsedTimeP = (TimeP - timePrevP) / 1000.0;
    PPID_d = kdP * ((PPID_error - Pprevious_error) / elapsedTimeP);
    PPID_value = PPID_p + PPID_i + PPID_d;
    PPID_value = constrain(PPID_value, 0, 255);
    analogWrite(airPump, PPID_value);
    Pprevious_error = PPID_error;
}
// Motor 1(Rod)
ISR(TIMER1_COMPA_vect) {
  static bool toggleMotor1 = false;
  static bool direction1 = false; // false = CCW, true = CW
  static unsigned long pauseStartTime1 = 0; // Start time for the pause
  static unsigned long pauseStartTime2 = 0; // Start time for the pause

  switch (Motor1State) {
    case 0: // Move CCW until the switch is hit
      if (digitalRead(RodSwitch) == LOW) {
        if(digitalRead(heatgunSwitch) == LOW){servo1.write(71);} //Flip servo up unless heatgun in the way
        // Stop motor
        digitalWrite(stepPin1, LOW);
        digitalWrite(enable1, LOW); // keep motor1 enabled
        digitalWrite(enable2, HIGH); // Dissable the motor2 driver
        Motor1State = 1; // Move to the next state: waiting for BottleIR to go high
        Motor2State = 1; //prepare Motor2State to screw bottle on
        delay(3000);
        pauseStartTime1 = millis();
        pauseStartTime2 = millis(); // Record the start time of the pause
        
        //reset stepcounts
        stepCount1 = 0;
        stepCount2 = 0;  
      }
      else {
        Motor2State = 2; //unscrew bottle at startup before moving backwards
        // // Toggle the step pin 1 to keep moving CCW
        if(millis() - pauseStartTime1 >= 3000){// wait 3 secs
          digitalWrite(dirPin1, LOW);
          OCR1A = 8200; // high means slow
          toggleMotor1 = !toggleMotor1;//move motor 1 until switch is hit (see Motor1State, 1)
          digitalWrite(stepPin1, toggleMotor1);
        }
        if (millis() - pauseStartTime2 >= 7000) { // Pause for 6 seconds
          OCR1A = 600; // low means fast
         
        }
      }
      break;
    case 1: // Wait for BottleIR to go low
      if (digitalRead(BottleIR) == LOW && digitalRead(RodSwitch) == LOW) {
        isrCounter++;
      }
      if (isrCounter >= 6000 && digitalRead(BottleIR) == LOW) { // Equivalent to a delay of 2000 ticks of ISR
        servo1.write(180); // bottlehugger move down
        digitalWrite(dirPin1, HIGH); // Set direction to CW
        digitalWrite(enable1, LOW); // Enable the motor driver
        Motor1State = 2; // Move to the next state: moving CW for 1 step
        stepCount1 = 0; // Reset step count for the new move
      } 
      break;
      
    case 2: // Move CW to lock into bottle for stepsPerRev1
      if (stepCount1 < stepsPerRev1-599) { //599 offset for small increments
        // Toggle the step pin to move CW
        OCR1A = 600; // fast frequency
        digitalWrite(enable1, LOW);
        digitalWrite(dirPin1, HIGH);
        toggleMotor1 = !toggleMotor1;
        digitalWrite(stepPin1, toggleMotor1);

        if (toggleMotor1) { // Only increment on HIGH state to avoid double counting
          stepCount1++;
        }
      } 
      else if ((stepCount1 >= stepsPerRev1-599) && (stepCount1 < stepsPerRev1)){// now moving slowly to screw on
        screwBottleOn = true; // flag motor 1 made it to bottle
        // Push cap on slightly with motor 1
          digitalWrite(dirPin1, HIGH); // Set direction to CW (screw on)
          OCR1A = 8200; // slow frequency
          toggleMotor1 = !toggleMotor1;//move motor 1 until switch is hit (see Motor1State, 1)
          digitalWrite(stepPin1, toggleMotor1);
          if (toggleMotor1) { // Only increment on HIGH state to avoid double counting
            stepCount1++;
        }
      }
      else {
        screwBottleOn = true; // flag motor 1 made it to bottle
        // Stop motor after stepsPerRev1
        digitalWrite(stepPin1, LOW);
        digitalWrite(enable1, LOW); // Keep pressure on the bottle!
        digitalWrite(dirPin1, LOW); // Set direction to CCW for the next cycle
      }
      break;
  }
}
// Motor 2(Screw)
ISR(TIMER4_COMPA_vect) {
  static bool toggleMotor2 = false;
  static bool toggleMotor1 = false;
  static unsigned long pauseStartTime = 0; // Start time for the pause
  static unsigned long delayStartTime = 0; // Start time for the 5-second delay

  switch (Motor2State) {
    case 1: // Move CW for .5 revolutions
      // Set up motor 2 to move CW
      digitalWrite(dirPin2, HIGH); // Set direction to CW (screw on)

    if (screwBottleOn){
      digitalWrite(enable2, LOW); // Enable the motor driver
      OCR4A = 1500; // high means slow
      if (stepCount2 < stepsPerRev2) { //Screw on motion
        toggleMotor2 = !toggleMotor2;
        digitalWrite(stepPin2, toggleMotor2);
        if (toggleMotor2) { // Only increment on HIGH state to avoid double counting
          stepCount2++;
        }
        delayStartTime = millis();//set timer for else if statement
      } 
      else if (stepCount2 >= stepsPerRev2) { //make sure it happens *after step count reaches starts incrementing
        // Stop motor after the steps are completed
        digitalWrite(stepPin2, LOW);
        digitalWrite(enable2, LOW); // Keep cap screwed on!'
        if (millis() - delayStartTime >= 1000) { // 3-second delay
          if(digitalRead(heatgunSwitch) == LOW){
            airCompressorFlag = true;
            heatgunFlag = true; // clear for heatgun motion
            servo1.write(71); //clear bottle hugger from heat gun
            pauseStartTime = millis(); //set timer in case 3
            Motor2State = 3; // move CW for heatgun motion
            //screwBottleOn = false;//dont screw anymore
          }
        }
      } 
    }
    break;
    

    case 2: // Screw Bottle OFF
      servo1.write(180);// keep servo down
      // Set up motor 2 to move CCW
      digitalWrite(dirPin2, LOW); // Set direction to CCW
      digitalWrite(enable2, LOW); // Enable the motor driver
      OCR4A = 1500; // high means slow
      if (stepCount2 < stepsPerRev2) { //added 1000 steps to ensure bottle screws off
        // Toggle the step pin to move CCW
        toggleMotor2 = !toggleMotor2;
        digitalWrite(stepPin2, toggleMotor2);
        if (toggleMotor2) { // Only increment on HIGH state to avoid double counting
          stepCount2++;
        }
        delayStartTime = millis();
      } 
      else {
        // Stop (slightly after) motor after the steps are completed
        if (millis() - delayStartTime >= 2000) { // 2-second delay
          if(heatgunSwitch == LOW){
            servo1.write(110);// Move the servo up
            digitalWrite(stepPin2, LOW);
            digitalWrite(enable2, HIGH); // Keep motor driver dissabled
            Motor1State = 0;
            Motor2State = 1; // Move to the next state
          }
        }
      }
      break;

      case 3: // Turning for Heatgun session
      //Power & turn motor2
        if(heatgunFlag){//As long as in heatgun mode . . .
          if (millis() - pauseStartTime >= 3000) { // Pause for 3 seconds
            if (stepCount3CW == 0) { 
              stepCount2 = 0; // Reset the step count when transition occurs
            }
            if(stepCount2 < 500){
              OCR4A = 1600; // high means slow
              digitalWrite(enable2, LOW); // enable screw cap motor
              digitalWrite(dirPin2, HIGH); // Set direction to CW
              toggleMotor2 = !toggleMotor2;
              digitalWrite(stepPin2, toggleMotor2);
              if (toggleMotor2) { // Only increment on HIGH state to avoid double counting
                stepCount2++;
              }
            }
            else{
              toggleMotor2 = toggleMotor2;
              digitalWrite(stepPin2, toggleMotor2);
              digitalWrite(enable2, HIGH); // dissable screw cap motor
            }
          }
        }
      break;
    
  }
}
// Motor 3(Heatgun)
ISR(TIMER3_COMPA_vect) {
  static bool toggleMotor3 = false;

  switch (Motor3State) {
    case 0: // Move CCW until the switch is hit
      if ((digitalRead(heatgunSwitch) == LOW) && (Motor3State == 0)) {
        // Stop motor
        digitalWrite(stepPin3, LOW);// Stop motor after hitting Zmin switch
        digitalWrite(enable3, LOW); // keep motor driver enabled
        Motor3State = 1; // prepare for state 1 (no heatgunFlag)
       
      } else {
        OCR3A = 200; //Go home fast
        // Toggle the step pin to keep moving CCW
        toggleMotor3 = !toggleMotor3;
        digitalWrite(stepPin3, toggleMotor3);
      }
      break;

    case 1: // Wait for pressure to reach above 20 psi
      if (pressure > 21.0 && heatgunFlag) {
        Motor3State = 2; // Move to the next state when pressure is above 20 psi
        stepCount3CW = 0;
      }
      break;

    case 2: //Turn CW
      if (stepCount3CW >= stepsPerRev3) { //if finished with CW steps..
        digitalWrite(stepPin3, LOW);
        stepCount3CCW = 0; // Reset the step count for CCW
        Motor3State = 3; // Move to the next state 
      } 
      else { // Set to turn CW
      digitalWrite(dirPin3, HIGH); 
      digitalWrite(enable3, LOW); // Enable the motor driver
      toggleMotor3 = !toggleMotor3;// Toggle set to true, to move CW
      digitalWrite(stepPin3, toggleMotor3);//Turn the motor
        if (toggleMotor3) { // Only increment on HIGH state to avoid double counting
            stepCount3CW++;
          }
      // Speed change to account for bottle curvature:
        if (stepCount3CW <= stepsPerRev3/3 && stepCount3CW >= 0){
          OCR3A = 3600; //Slower speed
        }
        else if (stepCount3CW <= 2*stepsPerRev3/3 && stepCount3CW > stepsPerRev3/3){
          OCR3A = 6600; //Slower speed
        }
        else if (stepCount3CW <= stepsPerRev3 && stepCount3CW > 2*stepsPerRev3/3){
          OCR3A = 3600; //Slower speed
        }
      }
      /////////////////////////////////////////////////
      break;

    case 3: // Turn CCW 
      OCR3A = 1600; //Faster speed
      if (stepCount3CCW >= stepsPerRev3) { //if finished with CCW steps..
        // Stop the motor
        digitalWrite(stepPin3, LOW);
        digitalWrite(enable3, HIGH); // Disable the motor driver
        stepCount3CW = 0; // Reset the step count for CW
        Motor3State = 2; // Move back to the CW state to continue the cycle
        
      } 
      else {
        digitalWrite(dirPin3, LOW); // Set to turn CCW
        digitalWrite(enable3, LOW); // Enable the motor driver
        // Toggle the step pin to move CCW
        toggleMotor3 = !toggleMotor3;
        digitalWrite(stepPin3, toggleMotor3); // Turn the motor
        if (toggleMotor3) { // Only increment on HIGH state to avoid double counting
          stepCount3CCW++;
        }
      }
    break;
  }
}

void updateDisplay(int menu_state){
    //Serial.print("***"); //to indicate display was updated
    switch(menu_state){
      case 0:
        // lcd.setCursor(0, 0);
        // if(readT<=.5*setT || setT <= readT){lcd.write(byte(4));}//Cold Therm
        // else if(readT >.5*setT && readT < setT){lcd.write(byte(3));}//Med Therm
        // else{lcd.write(byte(5));}//Hot Therm
        // lcd.print("~Bottle Prep");
        lcd.setCursor(14, 0); lcd.print("|Info|");
        // lcd.setCursor(0, 1);lcd.print("K");lcd.write(byte(0));    
        // lcd.setCursor(3, 1);lcd.print(kpT,1);  
        // lcd.setCursor(0, 2);lcd.print("K"); lcd.write(byte(1));      
        // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // lcd.setCursor(0, 3);lcd.print("K");  lcd.write(byte(2));     
        // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        lcd.setCursor(0, 3);lcd.print("Pres:");
        lcd.setCursor(6, 3);lcd.print(pressure,1); 
        lcd.setCursor(9, 3);lcd.write(byte(8));lcd.write(byte(9)); 
        // lcd.setCursor(0, 2);lcd.print("K");      
        // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // lcd.setCursor(0, 3);lcd.print("K");      
        // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        lcd.setCursor(0, 1); lcd.print("SetT:"); 
        lcd.setCursor(6, 1); lcd.print(setT,0); lcd.print((char)223);lcd.print("C  ");
        lcd.setCursor(0, 2); lcd.print("Temp:"); 
        //lcd.setCursor(6, 2); lcd.print(tempAvg(tempCalc(Thermistor1))); lcd.print((char)223);lcd.print("C  ");
        lcd.setCursor(6, 2); lcd.print(tempCalc(Thermistor1),1); lcd.print((char)223);lcd.print("C  ");
        break;

      case 1:
        // // lcd.setCursor(0, 0);
        // // if(readT<=.5*setT || setT <= readT){lcd.write(byte(4));}//Cold Therm
        // // else if(readT >.5*setT && readT < setT){lcd.write(byte(3));}//Med Therm
        // // else{lcd.write(byte(5));}//Hot Therm
        // // lcd.print("~Bottle Prep");
        // lcd.setCursor(14, 0); lcd.print("|Info|");
        // // lcd.setCursor(0, 1);lcd.print("K");lcd.write(byte(0));    
        // // lcd.setCursor(3, 1);lcd.print(kpT,1);  
        // // lcd.setCursor(0, 2);lcd.print("K"); lcd.write(byte(1));      
        // // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // // lcd.setCursor(0, 3);lcd.print("K");  lcd.write(byte(2));     
        // // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        // lcd.setCursor(0, 3);lcd.print("Pres:");
        // lcd.setCursor(6, 3);lcd.print(pressure,1); 
        // lcd.setCursor(9, 3);lcd.write(byte(8));lcd.write(byte(9)); 
        // // lcd.setCursor(0, 2);lcd.print("K");      
        // // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // // lcd.setCursor(0, 3);lcd.print("K");      
        // // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        // lcd.setCursor(0, 1); lcd.print("SetT:"); 
        // lcd.setCursor(6, 1); lcd.print(setT,0); lcd.print((char)223);lcd.print("C  ");
        // lcd.setCursor(0, 2); lcd.print("Temp:"); 
        // //lcd.setCursor(6, 2); lcd.print(tempAvg(tempCalc(Thermistor1))); lcd.print((char)223);lcd.print("C  ");
        // lcd.setCursor(6, 2); lcd.print(tempCalc(Thermistor1),1); lcd.print((char)223);lcd.print("C  ");
        // break;

      case 2:
      // lcd.setCursor(0, 0);
        // if(readT<=.5*setT || setT <= readT){lcd.write(byte(4));}//Cold Therm
        // else if(readT >.5*setT && readT < setT){lcd.write(byte(3));}//Med Therm
        // else{lcd.write(byte(5));}//Hot Therm
        // lcd.print("~Bottle Prep");
        lcd.setCursor(14,0);lcd.print("|SetT|"); 
        // lcd.setCursor(0, 1);lcd.print("K");lcd.write(byte(0));    
        // lcd.setCursor(3, 1);lcd.print(kpT,1);  
        // lcd.setCursor(0, 2);lcd.print("K"); lcd.write(byte(1));      
        // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // lcd.setCursor(0, 3);lcd.print("K");  lcd.write(byte(2));     
        // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        lcd.setCursor(0, 3);lcd.print("Pres:");
        lcd.setCursor(6, 3);lcd.print(pressure,1); 
        lcd.setCursor(9, 3);lcd.write(byte(8));lcd.write(byte(9)); 
        // lcd.setCursor(0, 2);lcd.print("K");      
        // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // lcd.setCursor(0, 3);lcd.print("K");      
        // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        lcd.setCursor(0, 1); lcd.print("SetT:"); 
        lcd.setCursor(6, 1); lcd.print(setT,0); lcd.print((char)223);lcd.print("C  ");
        lcd.setCursor(0, 2); lcd.print("Temp:"); 
        //lcd.setCursor(6, 2); lcd.print(tempAvg(tempCalc(Thermistor1))); lcd.print((char)223);lcd.print("C  ");
        lcd.setCursor(6, 2); lcd.print(tempCalc(Thermistor1),1); lcd.print((char)223);lcd.print("C  ");
        // lcd.setCursor(0, 0);
        // if(readT<=.5*setT || setT <= readT){lcd.write(byte(4));}//Cold Therm
        // else if(readT >.5*setT && readT < setT){lcd.write(byte(3));}//Med Therm
        // else{lcd.write(byte(5));}//Hot Therm
        // lcd.print("~PID");
        // lcd.setCursor(14,0);lcd.print("|SetT|");    
        // lcd.setCursor(0, 1);lcd.print("K");lcd.write(byte(0));    
        // lcd.setCursor(3, 1);lcd.print(kpT,1);  
        // lcd.setCursor(0, 2);lcd.print("K"); lcd.write(byte(1));      
        // lcd.setCursor(3, 2);lcd.print(kiT,1); 
        // lcd.setCursor(0, 3);lcd.print("K");  lcd.write(byte(2));     
        // lcd.setCursor(3, 3);lcd.print(kdT,1); 
        // lcd.setCursor(9, 2); lcd.print("SetT:"); 
        // lcd.setCursor(15, 2); lcd.print(setT,0); lcd.print((char)223);lcd.print("C  ");
        // lcd.setCursor(9, 3); lcd.print("Temp:"); 
        // //lcd.setCursor(15, 3); lcd.print(tempAvg(tempCalc(Thermistor1))); lcd.print((char)223);lcd.print("C  ");
        // lcd.setCursor(6, 2); lcd.print(tempCalc(Thermistor1),1); lcd.print((char)223);lcd.print("C  ");

        break;

      default:
      lcd.setCursor(5,0); lcd.print("|SPOOL-IT|");
      break;
    }
}

void loop() {
  static unsigned long lastTempPIDUpdate = 0;
  static unsigned long lastPsiPIDUpdate = 0;
  // [ Heating element PID ]
  if ((menu_state == 1 || menu_state == 0) && millis() - lastTempPIDUpdate >= intervalT) {
    lastTempPIDUpdate = millis();
    updateTempPID();
  }
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastSensorRead = 0;
  if(LCDclear == 1){lcd.clear();LCDclear = 0;}
  updateDisplay(menu_state);
  // [ Read Pressure ]
    if (millis() - lastSensorRead >= 1000) {
      lastSensorRead = millis();
      if (sensorDetected) {
        pressure = mpr.readPressure();
      } else {
        pressure = 0; // Default/error value
        Serial.println("Pressure read error");
      }
    }
    // [ Air compressor PID ]
    if (airCompressorFlag && millis() - lastPsiPIDUpdate >= intervalP){
      Serial.println("PUMP PUMP PUMP");
      lastPsiPIDUpdate = millis();
      updatePsiPID();
  
    }
    if (millis() - lastDisplayUpdate >= 500) {
      lastDisplayUpdate = millis();
      updateDisplay(menu_state);
    }

    

  if(menu_state==0)
  {}
  //CheckiTng Motor PID
  if(menu_state == 1)
  {}
  //Setting the temperature
  if(menu_state == 2)
  {}

  Serial.println("______________________");
    Serial.print("Air Compressor Flag: ");Serial.println(airCompressorFlag);
    Serial.print("Motor1 State: ");Serial.println(Motor1State);
    Serial.print("Motor2 State: ");Serial.println(Motor2State);
    Serial.print("Motor3 State: ");Serial.println(Motor3State);
    // Serial.print("--Steps CW:  ");Serial.println(stepCount3CW);
    // Serial.print("--Steps CCW: ");Serial.println(stepCount3CCW);
    Serial.print("Temperature: ");Serial.print(tempAvg(tempCalc(Thermistor1)));Serial.println(" C");
    Serial.print("Pressure: ");Serial.print(mpr.readPressure());Serial.println(" psi");
    //delay(1000);

}
// Encoder Interrupt
ISR(PCINT0_vect){
  if(menu_state==2)
  {
    clk_State =   (PINL & B00000001); // Pin D49 
    dt_State  =   (PINB & B00000100); // Pin D51
    if (clk_State != Last_State){     
      // If the data state is the same to the clock state, that means the encoder is rotating clockwise
      if (dt_State != clk_State) { 
        setT = setT-10 ;
      }
      else {
        setT = setT+10;
      } 
    }
    Last_State = clk_State; // Updates the previous state of the clock with the current state
  } 
  if(menu_state==0 || menu_state==1)
    {
    LCDclear = 1;
    clk_State =   (PINL & B00000001); // Pin D49 is HIGH?
    dt_State  =   (PINB & B00000100); // pin 51
    if (clk_State != Last_State){     
      // If the data state is different to the clock state, that means the encoder is rotating CCW
      if (dt_State != clk_State) { 
        menu_state++;
        menu_state = menu_state % 2;
      }
      else {
        menu_state++;
        menu_state = menu_state % 2;
      } 
    }
    Last_State = clk_State; // Updates the previous state of the clock with the current state
  } 

   // Push button was pressed
  if (PINB & B00000001) { // Pin D53 is HIGH
    button_pressed = true; 
  } 
  else if(button_pressed && menu_state == 2) {
    menu_state = 0; 
    LCDclear = 1;
    button_pressed = false; 
    delay(900);
  } 
  else if(button_pressed && (menu_state == 0 || menu_state == 1)) {
    menu_state = 2;  
    LCDclear = 1;
    button_pressed = false;
    delay(900);
  }
  
}