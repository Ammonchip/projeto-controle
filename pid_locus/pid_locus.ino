#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp = 12.4200, Ki = 0.9999, Kd = 14.6300;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

/**
 * The digital pin that receives input from the ultrasonic sensor.
 */
const int echoPin = 2;
/**
 * The digital pin that sends output to the ultrasonic sensor.
 */
const int trigPin = 4;


#define motor 11 // 1 bomba 
#define motor2 9//2 bomba

void runMotor(int time_us, double pwm_value); //função para controle do motor

unsigned short pwm_value = 0x00; //armazena valor do duty cycle para PWM 

void setup() {
  pinMode(motor, OUTPUT); //saida para o motor
  digitalWrite(motor, LOW); // motor inicia desligado

  pinMode(motor2, OUTPUT); //saida para o motor 2
  digitalWrite(motor2, LOW); // motor 2 inicia desligado

  pinMode(echoPin, INPUT);  // Register echoPin for receiving input
  pinMode(trigPin, OUTPUT);  // Register trigPin for sending output
  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 10;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);  // Begin serial communication to receive data from the ultrasonic sensor
}

void loop() {
  // Send a short low pulse to ensure a clean high one.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a ten-second high pulse.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Store the high pulse's duration.
  const long duration = pulseIn(echoPin, HIGH);

  // Calculate and print the distance to the target.
  const double distance = 17 - microsecondsToDistance(duration);
  Serial.print("Nivel agua tanque 2: ");
  Serial.print(distance, 2);
  Serial.print(" cm   -----   ");
  Input = distance;
  myPID.Compute();
  Serial.print("Output: ");
  Serial.print(Output, 2);
  Serial.print("   ---   Voltage: ");
  Serial.print(Output/17, 2);
  Serial.print(" V   -----   ");
  Serial.print("error: ");
  Serial.print((Setpoint - distance)*10, 2);
  Serial.println("%");
  runMotor(4000,Output);
}


void runMotor(int time_us, double pwm_value)
{
  
  analogWrite(motor, pwm_value); //atualiza PWM do motor
  analogWrite(motor2, pwm_value);
  delayMicroseconds(time_us); //pelo tempo determinado no parâmetro
  
} //end runMotor

/**
 * @param microseconds a number of microseconds
 * @return the conversion of the provided microseconds into a distance
 */
const double microsecondsToDistance(const long microseconds) {
  // Initialize m and b to their respective values in the formula, y = mx + b.
  // y = distance, x = time (in microseconds).
  const double m = 0.01739163974;
  const double b = -0.2610713059;
  
  return m * microseconds + b;
}