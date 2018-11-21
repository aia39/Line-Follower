# Line-Follower
For the enthusiast who loves to build simple robot.This code is executable in arduino IDE



#define   inA         2
#define   inB         3
#define   inC         4
#define   inD         7
#define   enA         5
#define   enB         6

int leftBaseSpeed   = 130;
int rightBaseSpeed  = 130;
int maxSpeed        = 255;

// SENSOR PARAMETERS

#define NUM_SENSORS   5

int thresholds[NUM_SENSORS] = {450, 450, 450,  450, 450};
int ledPins[5]              = {8, 9, 10, 11, 12};
int sensorValues[NUM_SENSORS], sValues[NUM_SENSORS], lastSensor, psValues[NUM_SENSORS], mode,distR = 0, distF = 0;// durationF = 0, durationR = 0;
unsigned long durationF=0,durationR=0;
// PID PARAMETERS

float kp            = 48.0;
float kd            = 4.7;
float ki            = 0.75;
int prevError = 0, error = 0;
float p, i = 0, d;

void setup() {
  // put your setup code here, to run once:

  //Initialize Motor Pins
  motorInit();
  //Initialize Other Variables and Pins
  otherInit();
}

void loop() {
  //if(distF<value&&disR<value) wallFollow();
  lineFollow();
}


void motorInit()
{
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  //Setting The Motor to zero speed
  digitalWrite(inA, HIGH);
  digitalWrite(inB, HIGH);
  digitalWrite(inC, HIGH);
  digitalWrite(inD, HIGH);
}

void otherInit()
{
  //for (int i = 0; i < 5; i++) pinMode(ledPins[i], OUTPUT);
 
  lastSensor = 0;
  prevError = 0;
  Serial.begin(9600);
}
int readSensor()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //Read the raw sensor values
    sensorValues[i] = analogRead(i);
    //Serial.print(i+1); Serial.print(" : "); Serial.println(sensorValues[i]); delay(700);
    //Covert them into digital readings
    //And turning the corresponding LED on or off
    if (sensorValues[i] > thresholds[i])
    {
      sValues[i] = 0;
      //digitalWrite(ledPins[i], LOW);
    }
    else
    {
      sValues[i] = 1;
      //digitalWrite(ledPins[i], HIGH);
    }
    //detectPos();
    /*if (sensorValues[i] > thresholds[i])
    {
      sValues[i] = psValues[i] + mode;
    }
    else
    {
      sValues[i] = psValues[i] - mode;
    }*/
  }
  if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 1 && sValues[3] == 0 && sValues[4] == 0)
  {
    error = 0;
  }
  else if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 0 && sValues[3] == 1 && sValues[4] == 0)
  {
    error = 1;
  }
  else if (sValues[0] == 0 && sValues[1] == 1 && sValues[2] == 0 && sValues[3] == 0 && sValues[4] == 0)
  {
    error = -1;
  }
  else if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 0 && sValues[3] == 0 && sValues[4] == 1)
  {
    error = 3;
    lastSensor = 2;
  }
  else if (sValues[0] == 1 && sValues[1] == 0 && sValues[2] == 0 && sValues[3] == 0 && sValues[4] == 0)
  {
    error = -3;
    lastSensor = 1;
  }
  else if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 1 && sValues[3] == 1 && sValues[4] == 0)
  {
    error = 2;
  }
  else if (sValues[0] == 0 && sValues[1] == 1 && sValues[2] == 1 && sValues[3] == 0 && sValues[4] == 0)
  {
    error = -2;
  }
  else if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 1 && sValues[3] == 1 && sValues[4] == 1)
  {
    error = 4;
    lastSensor = 2;
  }
  else if (sValues[0] == 1 && sValues[1] == 1 && sValues[2] == 1 && sValues[3] == 0 && sValues[4] == 0)
  {
    error = -4;
    lastSensor = 1;
  }
  else if (sValues[0] == 0 && sValues[1] == 1 && sValues[2] == 1 && sValues[3] == 1 && sValues[4] == 1)
  {
    error = 5;
    lastSensor = 2;
  }
  else if (sValues[0] == 1 && sValues[1] == 1 && sValues[2] == 1 && sValues[3] == 1 && sValues[4] == 0)
  {
    error = -5;
    lastSensor = 1;
  }
  else if (sValues[0] == 0 && sValues[1] == 0 && sValues[2] == 0 && sValues[3] == 0 && sValues[4] == 0)
  {
    if (lastSensor == 1)
    {
      error = -6;
    }
    else if (lastSensor == 2)
    {
      error = 6;
    }
  }
 

  return error;
}


void lineFollow()
{
  int pid;

  //Read the sensor values and calculate error
  error = readSensor();
 
  p = error * kp;
  i = (i + error) * ki; //i=i+ilast;
  d = (error - prevError) * kd;
  pid = int(p + i + d);

  //lasti=i;
  //motor_control();
  wheel(leftBaseSpeed - pid, rightBaseSpeed + pid);
  prevError = error;
  if (error - prevError == 0) delay(10);
 }

void wheel(int leftSpeed, int rightSpeed)
{
  if ( leftSpeed == 0)
  {
    digitalWrite(inC, 1);
    digitalWrite(inD, HIGH);
  }
  if ( leftSpeed > 0)
  {
    digitalWrite(inC, 1);
    digitalWrite(inD, 0);
    //delay(5);
  }
  else if ( leftSpeed < 0)
  {
    digitalWrite(inC, 0);
    digitalWrite(inD, 1);
    //delay(5);
  }

  if (rightSpeed == 0)
  {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);
  }
  if ( rightSpeed > 0)
  {
    digitalWrite(inA, 1);
    digitalWrite(inB, 0);
    //delay(10);
  }
  else if ( rightSpeed < 0)
  {
    digitalWrite(inA, 0);
    digitalWrite(inB, 1);
    //delay(10);
  }
  if (abs(leftSpeed) > maxSpeed) leftSpeed = maxSpeed;
  if (abs(rightSpeed) > maxSpeed) rightSpeed = maxSpeed;

  analogWrite(enA, abs(rightSpeed));
  analogWrite(enB, abs(leftSpeed));
}
