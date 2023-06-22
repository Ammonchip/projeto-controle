/**
 * The digital pin that receives input from the ultrasonic sensor.
 */
const int echoPin = 2;
/**
 * The digital pin that sends output to the ultrasonic sensor.
 */
const int trigPin = 4;

#define motor 12 // MotorDC

void runMotor(int time_us); //função para controle do motor

void tankModel();

unsigned short pwm_value = 0x00; //armazena valor do duty cycle para PWM 

void tankModel() {

}

void runMotor(int time_us) {
  if (pwm_value == 0xFF) {
    pwm_value = 0xFF;

    digitalWrite(motor, HIGH);
  }
  else {
    analogWrite(motor, pwm_value); // atualiza PWM do motor

    pwm_value++; //incrementa duty cycle
  }

  delayMicroseconds(time_us); 
}


void setup() {
  pinMode(motor, OUTPUT); //saida para o motor
  digitalWrite(motor, LOW); // motor inicia desligado
  pinMode(echoPin, INPUT);  // Register echoPin for receiving input
  pinMode(trigPin, OUTPUT);  // Register trigPin for sending output
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
  const double distance = microsecondsToDistance(duration);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  runMotor(4000);
}

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