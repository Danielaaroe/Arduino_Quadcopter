
//using 3pc ADXL320/ADXRS610 IMU +/-300deg/s gyro and +/- 5g accelerometer

#include<Servo.h>
#include <PinChangeInt.h>

//define ports
//outputs
#define MOTOR_0_PIN 6
#define MOTOR_1_PIN 9
#define MOTOR_2_PIN 10
#define MOTOR_3_PIN 11

//digital inputs
#define THROTTLE_PIN 2
#define PITCH_PIN 4
#define ROLL_PIN 7
#define YAW_PIN 8
#define BLOCK_PIN 12 //Connected to switch driven RC output and used to block motors from running unintentionally.

//analog inputs
#define X_RAW_PIN 0
#define Y_RAW_PIN 1
#define Z_RAW_PIN 2
#define X_RATE_PIN 3
#define Y_RATE_PIN 4
#define Z_RATE_PIN 5

//PID
#define P_TERM_MAX 3
#define P_TERM_MIN -3
#define I_TERM_MAX 3
#define I_TERM_MIN -3
#define PID_MAX 10
#define PID_MIN -10

//interrupts
#define THROTTLE_FLAG 1
#define PITCH_FLAG 2
#define ROLL_FLAG 4
#define YAW_FLAG 8

//forskjellig
#define countermax 10
#define readcountermax 3 //the max number of channels to be read(channel number starting from channel 0)

//Analog readings and angles.
double dt = 0;
long oldtime, thistime = 0; //dt and help variables
int bias[6] = {0, 0, 0, 0, 0, 0}; //analog offsets with respect to port number
double xacc, yacc, zacc, xrate, yrate, zrate = 0; //analog readings
double  xaccangle, //accelerometerangle around the x-axis
        yaccangle, //accelerometerangle arond the y-axis
        xangle, //filtered angle around the x-axis
        yangle = 0.0; //filtered angle around the y-axis

// Incoming signals 2/4/7/8
volatile int ThrottleInShared = 0;
volatile int PitchInShared = 0;
volatile int RollInShared = 0;
volatile int YawInShared = 0;
volatile int BlockInShared = 0;


// Incoming signals 2/4/7/8 - local variables to avoid ISR changing this word during loop read.
int throttleval;
int pitchval;
int rollval;
int yawval;
int blockval;

long ulThrottleStart;
long ulPitchStart;
long ulRollStart;
long ulYawStart;
long ulBlockStart;

//motors 6/9/10/11
Servo motor0, motor1, motor2, motor3;

// parameters
double pitchcontribution = 0;
double rollcontribution = 0;
double yawcontribution = 0;

//PID variables 0 = pitch, 1 = roll, 2 = yaw.
//double err_pv[] ={0,0,0}; Not needed due to shift of pid type to avoid derivative spike
//double olderr_pv[] ={0,0,0};
//constants
double Kp[] = {0.1, 0.1, 2}; //tested with Pcontroller. marginally stable while hanging somewhat underneath thread. 0.05 ok 0.1 ustabilt(oscillerende)
double Ki[] = {0.3, 0.03, 1};
double Kd[] = {1, 1, 1};
double PV[] = {0, 0, 0};
double PV_old[] = {0, 0, 0};
//values
//double proportional[] ={0,0,0}; lages i funksjonen
double integral[3] = {0, 0, 0};
double derivative[] = {0, 0, 0};
int counter = 0;


//Kalman variables
int F_x[] = {1, 1}; //transition matrix -> k-1 coefficient
int F_u[] = {1, 1}; //control matrix - speed coefficient
int F_n[] = {1, 1}; //pert. matrix
int H[] = {1, 1}; //measurement matrix
double Q[] = {0.001, 0.001}; //speed covariance == speed noise^2
double R[] = {30, 30};  // position covariance == position noise^2

double x[] = {1, 1}; // estimate initial state
double P[] = {100, 100}; // initial estimate trustability
double e[] = {1, 1}; //difference between estimate and measurement(filtered angle and accelerometerangle)
double E[] = {1, 1};
double z[] = {1, 1};
double Z[] = {1, 1};
double K[] = {1, 1};



void setup() {

delay(10);

  //Start serial
  Serial.begin(115200);

  //Declare control-inputs as inputs
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);

  //attach motors
  motor0.attach(MOTOR_0_PIN);
  motor1.attach(MOTOR_1_PIN);
  motor2.attach(MOTOR_2_PIN);
  motor3.attach(MOTOR_3_PIN);

  //attach interrupts
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_PIN, calcThrottle, CHANGE);
  PCintPort::attachInterrupt(PITCH_PIN, calcPitch, CHANGE);
  PCintPort::attachInterrupt(ROLL_PIN, calcRoll, CHANGE);
  PCintPort::attachInterrupt(YAW_PIN, calcYaw, CHANGE);
  PCintPort::attachInterrupt(BLOCK_PIN, calcBlock, CHANGE);

}
void loop() { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //delay(10);

  noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
  throttleval = ThrottleInShared;
  pitchval = PitchInShared;
  rollval = RollInShared;
  yawval = YawInShared;
  blockval = BlockInShared;

  interrupts();



  updateDt();
  getAnalog();

  getAngles();
  getControl();

  pitchcontribution = PID(0, pitchval, yangle);
  rollcontribution = PID(1, rollval, xangle);
  // yawcontribution = PID(2, yawval, zrate);

  Serial.println(xangle);
  //Serial.print("   ");
  //Serial.println(yangle);


  double motor_0 = 45 + (throttleval - pitchcontribution) * 0.93; //front
  double motor_1 = 45 + throttleval - rollcontribution; //right
  double motor_2 = 45 + throttleval + pitchcontribution; //rear
  double motor_3 = 45 + throttleval + rollcontribution; //left


  setMotors(motor_0, motor_1, motor_2, motor_3, blockval);
  //Serial.print(motor_0);
  //Serial.print("   ");
  //Serial.print(motor_1);
  //Serial.print("   ");
  //Serial.print(motor_2);
  //Serial.print("   ");
  //Serial.println(motor_3);

}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateDt() {
  thistime = micros();
  dt = (thistime - oldtime) / 1000.0;
  oldtime = thistime;
}

void calculateBias() {
  int tempbias = 0;
  for (int i = 0; i <= 5; i++) {
    for (int j = 0; j <= 50; j++) {
      tempbias += analogRead(i);
    }
    tempbias = tempbias / 50;
    bias[i] = tempbias;
  }
}

double kalman(int axis_no, double y, double u, int F_u) {
  //estimate - prediction
  x[axis_no] = F_x[axis_no] * x[axis_no] + F_u * u;
  P[axis_no] = F_x[axis_no] * P[axis_no] + F_n[axis_no] * Q[axis_no]; //P = F_x * P * F_x + F_n * Q * F_n;[axis_no]

  // correction
  e[axis_no] = H[axis_no] * x[axis_no];
  E[axis_no] = H[axis_no] * P[axis_no]; // E = H * P * H;
  z[axis_no] = y - e[axis_no];
  Z[axis_no] = R[axis_no] + E[axis_no];
  K[axis_no] = P[axis_no] * H[axis_no] * 1 / Z[axis_no];
  x[axis_no] = x[axis_no] + K[axis_no] * z[axis_no];
  P[axis_no] = P[axis_no] - K[axis_no] * H[axis_no] * P[axis_no];

 //Serial.println(x[0]);
   
  return x[axis_no];

 
}


void getControl() { //vurder senere hva disse verdiene bør være

  throttleval = map(throttleval, 1050, 1850, 0, 90); //bør man kunne gi full gass? hva med regulering?
  pitchval = map(pitchval, 1050, 1850, -45, 45); //bør nok være slik
  rollval = map(rollval, 1050, 1850, -45, 45);    //bør nok være slik
  yawval = map(yawval, 1050, 1850, -45, 45); //bør sikkert være en mindre verdi typ +/- 20
}

void getAnalog() { //reads the analog ports to get all the accelerometer and gyroscope data.

  xacc = analogRead(X_RAW_PIN) - 509;
  yacc = analogRead(Y_RAW_PIN) - 500;
  zacc = analogRead(Z_RAW_PIN) - 512;
  xrate = analogRead(X_RATE_PIN) - 512;
  yrate = analogRead(Y_RATE_PIN) - 502;
  zrate = analogRead(Z_RATE_PIN) - 512;

  // Serial.print(xacc);
  //   Serial.print(" ");
  // Serial.print(yacc);
  //   Serial.print(" ");
  //   Serial.println(zacc);
}

void getAngles() {

  //angle around the y-axis --må dobbelsjekkes
  yaccangle = atan2(xacc, sqrt(yacc * yacc + zacc * zacc)) * RAD_TO_DEG;
  yrate = map(yrate, -512, 512, -0.00300, 0.00300); ///deg/sek
  yangle = kalman(0, yaccangle, yrate, dt);

  //angle around the x-axis --må dobbelsjekkes
  xaccangle = atan2(-yacc, sqrt(xacc * xacc + zacc * zacc)) * RAD_TO_DEG;
  xrate = map(xrate, -512, 512, -0.00300, 0.00300);//deg/sek
  xangle = kalman(1, xaccangle, xrate, dt);

  zrate = map(zrate, 0, 1023, -300, 300);
  
  //Serial.print(xaccangle);
  //Serial.print("  ");
  //Serial.println(xangle);
  //Serial.print("  ");
  //Serial.println(xrate);
}

double PID(int index, double reference, double measured) {
  double Pterm = 0;
  double Iterm = 0;
  double Dterm = 0;
  double err = 0;

  err = reference - measured;

  PV_old[index] = PV[index];
  PV[index] = -measured;
  derivative[index] = (PV[index] - PV_old[index]) / dt;

  integral[index] += Ki[index] * err * dt;
  integral[index] = constrain(integral[index], I_TERM_MIN, I_TERM_MAX);

  Pterm = Kp[index] * err;
  Iterm = integral[index];
  Dterm = Kd[index] * derivative[index];

  Pterm = constrain(Pterm, P_TERM_MIN, P_TERM_MAX);

  double total = Pterm + Iterm/* + Dterm*/;

  total = constrain(total, PID_MIN, PID_MAX);

  return total;
}


void setMotors(double motor0val, double motor1val, double motor2val, double motor3val, int block) {
  if (block < 1600) {
    motor0.write(40);
    motor1.write(40);
    motor2.write(40);
    motor3.write(40);
  } else {
    motor0.write(motor0val);
    motor1.write(motor1val);
    motor2.write(motor2val);
    motor3.write(motor3val);
  }

}
void updateCounter() {
  counter = counter++;
  if (counter == countermax) {
    counter = 0;
  }
}

void getParams() { //a function that reads serial for updating parameters in the PID, for starters.
  if (Serial.available() > 0) {
  }
}

// ISR
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if (digitalRead(THROTTLE_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    ThrottleInShared = (micros() - ulThrottleStart);

  }
}
void calcPitch()
{

  if (digitalRead(PITCH_PIN) == HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    PitchInShared = (micros() - ulPitchStart);
  }
}
void calcRoll()
{

  if (digitalRead(ROLL_PIN) == HIGH)
  {
    ulRollStart = micros();
  }
  else
  {
    RollInShared = (micros() - ulRollStart);
  }
}
void calcYaw()
{

  if (digitalRead(YAW_PIN) == HIGH)
  {
    ulYawStart = micros();
  }
  else
  {
    YawInShared = (micros() - ulYawStart);
  }
}
void calcBlock()
{

  if (digitalRead(BLOCK_PIN) == HIGH)
  {
    ulBlockStart = micros();
  }
  else
  {
    BlockInShared = (micros() - ulBlockStart);
  }
}
