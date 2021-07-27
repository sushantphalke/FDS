
#include <XBOXONE.h>
#include <AccelStepper.h>
#define dirPin 10         // CW
#define stepPin 11        // CLK
#define stepsPerRevolution 1600

#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXONE Xbox(&Usb);
#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3
#include <Sabertooth.h>
Sabertooth ST(128, Serial1); // The Sabertooth is on address 128. We'll name its object ST.



unsigned long int timer = 0;
bool flag = 0;
double Setpoint = 0.0, Input = 0.0, Output = 0.0;
double Kp = 1.6, Ki = 0, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



unsigned long int timer1 = 0;
double Setpoint1 = 0.0, Input1 = 0.0, Output1 = 0.0;
double Kp1 = 5, Ki1 = 0, Kd1 = 0.1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);




bool Y_button = LOW;



volatile signed int  counter_grb = 0;
float i = 0;



volatile signed int   counter_leadscr = 0;
int flag_leadscr = 0;
int flag_grb = 0;


void setup() {
  Serial1.begin(19200);
  Serial.begin(19200);

  ST.autobaud();

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);

  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(10000);

  myPID.SetOutputLimits(-30, 30);
  myPID1.SetOutputLimits(-125, 125);
  attachInterrupt(digitalPinToInterrupt(19), counter_leadscr_func, RISING);
  attachInterrupt(digitalPinToInterrupt(2), counter_grb_func, RISING);


  //initialize the variables we're linked to
  Input = counter_grb;
  Setpoint = 0;

  //initialize the variables we're linked to
  Input1 =  counter_leadscr;
  Setpoint1 = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);


  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}
bool Y_local = 0;
bool Y_flag = 0;
bool Y_loop;
int Y_counter = 0;
bool X_local = 0;
bool X_flag = 0;
bool X_loop;
int X_counter = 0;
bool A_local = 0;
bool A_flag = 0;
bool A_loop;
int A_counter = 0;
bool B_local = 0;
bool B_flag = 0;
bool B_loop;
int B_counter = 0;



void loop() {
  Usb.Task();
  if (Xbox.XboxOneConnected) {


    //////

    Y_loop =  Xbox.getButtonClick(Y);
    if (Y_flag == 0 && Y_loop == 1) {
      Y_local = 1;
      Y_flag = 1;
      Y_counter = Y_counter + 1;
      Serial.print("Y_local = 1");
    }
    if (Y_loop == 0 && Y_flag == 1) {
      Y_local = 0;
      Y_flag = 0;
      Serial.print("Y_local = 0");
    }
    Serial.print("Y_counter : ");
    Serial.print(Y_counter);

    ////


    X_loop =  Xbox.getButtonClick(X);
    if (X_flag == 0 && X_loop == 1) {
      X_local = 1;
      X_flag = 1;
      X_counter = X_counter + 1;
      Serial.print("X_local = 1");
    }
    if (X_loop == 0 && X_flag == 1) {
      X_local = 0;
      X_flag = 0;
      Serial.print("X_local = 0");
    }
    Serial.print("X_counter : ");
    Serial.print(X_counter);

    ////

    A_loop =  Xbox.getButtonClick(A);
    if (A_flag == 0 && A_loop == 1) {
      A_local = 1;
      A_flag = 1;
      A_counter = A_counter + 1;
      Serial.print("A_local = 1");
    }
    if (A_loop == 0 && A_flag == 1) {
      A_local = 0;
      A_flag = 0;
      Serial.print("A_local = 0");
    }
    Serial.print("  A_counter : ");
    Serial.println(A_counter);

    ////

    B_loop =  Xbox.getButtonClick(B);
    if (B_flag == 0 && B_loop == 1) {
      B_local = 1;
      B_flag = 1;
      B_counter = B_counter + 1;
      Serial.print("B_local = 1");
    }
    if (B_loop == 0 && B_flag == 1) {
      B_local = 0;
      B_flag = 0;
      Serial.print("B_local = 0");
    }
    Serial.print("  B_counter : ");
    Serial.println(B_counter);

    ////


    timer = millis();
    Input = counter_grb;
    myPID.Compute();

    /////



    timer1 = millis();
    Input1 =  counter_leadscr;
    myPID1.Compute();

    ////

    if (counter_grb < 440 && counter_grb > -30) {

      if ( X_counter == 0) {
        Setpoint = 0 ;
        ST.motor(2, Output);
      }

      if (X_counter == 1 ) {
        Serial.print("grb_1  ");
        Setpoint = 20 ; //20  //390  //-410
        ST.motor(2, Output);

      }

      if ( X_counter == 2) {
        Serial.print("grb_2  ");
        Setpoint = 390 ;  //390  //0
        ST.motor(2, Output);

      }
      if ( X_counter == 3) {
        Serial.print("grb_3   ");
        Setpoint = 0 ;
        ST.motor(2, Output);
        X_counter = 0;

      }
    }
    else {
      myPID.SetOutputLimits(0 , 0);
    }
    //    Serial.print("\toutput: ");
    //    Serial.print(Output);
    //    Serial.print("Enccounter: ");
    //    Serial.println(counter_grb);

    //////

    if (counter_leadscr < 1300 && counter_leadscr >= -20) {

      if ( A_counter % 2 == 0) {
        Setpoint1 = 0 ;
        Serial.print("Inside data : ");
        ST.motor(1, Output1);
      }

      if (A_counter % 2 == 1) {
        Serial.print("  leadscr_1  ");
        Setpoint1 = 1249 ;
        ST.motor(1, Output1);


      }


      Serial.print("\toutput_leadscr: ");
      Serial.print(Output1);
      Serial.print(" Enccounter_leadscr: ");
      Serial.print( counter_leadscr);

    }
    else {
      myPID1.SetOutputLimits(0 , 0);
      ST.motor(1, 0);

    }


    /////

    if (B_counter % 10 == 1) {
      stepper.moveTo(453);
      stepper.runToPosition();
    }
    if (B_counter % 10  == 2) {
      stepper.moveTo(462 * 2 );
      stepper.runToPosition();
    }
    if (B_counter % 10  == 3) {
      stepper.moveTo(470 * 3);
      stepper.runToPosition();
    }
    if (B_counter % 10  == 4) {
      stepper.moveTo(1605);
      stepper.runToPosition();
    }
    if (B_counter % 10 == 5) {
      stepper.moveTo(1855);
      stepper.runToPosition();
    }
    if (B_counter % 10  == 6) {
      stepper.moveTo(2080);
      stepper.runToPosition();
    }
    if (B_counter % 10  == 7) {
      int count2 = 2080;
      stepper.moveTo(count2 + (453 ));
      stepper.runToPosition();
    }
    if (B_counter % 10  == 8) {
      int count2 = 2080;
      stepper.moveTo(count2 + (453 * 2 ));
      stepper.runToPosition();
    }
    if (B_counter % 10 == 9) {
      int count2 = 2080;
      stepper.moveTo(count2 + (453 * 3 ));
      stepper.runToPosition();

    }
    if (B_counter % 10 == 0) {
      
      stepper.moveTo(0);
      stepper.runToPosition();

    }

    /////


    if (Xbox.getButtonClick(UP))
      Serial.println(F("Up"));
    if (Xbox.getButtonClick(DOWN))
      Serial.println(F("Down"));
    if (Xbox.getButtonClick(LEFT))
      Serial.println(F("Left"));
    if (Xbox.getButtonClick(RIGHT))
      Serial.println(F("Right"));

    if (Xbox.getButtonClick(START))
      Serial.println(F("Start"));
    if (Xbox.getButtonClick(BACK))
      Serial.println(F("Back"));
    if (Xbox.getButtonClick(XBOX))
      Serial.println(F("Xbox"));
    if (Xbox.getButtonClick(SYNC))
      Serial.println(F("Sync"));

    if (Xbox.getButtonClick(L1))
      Serial.println(F("L1"));
    if (Xbox.getButtonClick(R1))
      Serial.println(F("R1"));
    if (Xbox.getButtonClick(L2))
      Serial.println(F("L2"));
    if (Xbox.getButtonClick(R2))
      Serial.println(F("R2"));
    if (Xbox.getButtonClick(L3))
      Serial.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      Serial.println(F("R3"));


    if (Xbox.getButtonClick(A))

      Serial.println(F("A"));
    if (Xbox.getButtonClick(B))
      Serial.println(F("B"));










    delay(1);
  }
}
void counter_grb_func() {
  if (digitalRead(2)) {
    if (digitalRead(3)) {
      --counter_grb;
    } else {
      ++counter_grb;
    }
  }
}

void  counter_leadscr_func() {
  if (digitalRead(19)) {
    if (digitalRead(20)) {
      -- counter_leadscr;
    } else {
      ++ counter_leadscr;
    }
  }
}
