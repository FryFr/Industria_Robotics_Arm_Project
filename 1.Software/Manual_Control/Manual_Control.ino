#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

// Definición de los límites de pulso para los servos
const int SERVOMIN = 150; // Pulso 'ínimo' en microsegundos
const int SERVOMAX = 600; // Pulso 'áximo' en microsegundos

// Limites específicos de cada servo
enum ServoLimits {
  SERVO0_MIN = 0,
  SERVO0_MAX = 180,
  SERVO1_MIN = 0,
  SERVO1_MAX = 85,
  SERVO2_MIN = 0,
  SERVO2_MAX = 90,
  SERVO3_MIN = 2,
  SERVO3_MAX = 90,
  SERVO4_MIN = 0,
  SERVO4_MAX = 80
};

bool automationActive = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando PCA9685...");
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia PWM estándar para servos
  
  Serial.println("Presiona 'Y' para comenzar la automatización o 'S' para detenerla.");
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();
    
    if (input == 'Y' || input == 'y') {
      automationActive = true;
      executeSequence();
    }
    
    if (input == 'S' || input == 's') {
      automationActive = false;
      Serial.println("Automatización detenida.");
    }
  }
}

void moveServo(int servo, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulseLength);
  Serial.print("Moviendo servo ");
  Serial.print(servo);
  Serial.print(" a ");
  Serial.print(angle);
  Serial.println(" grados.");
}

void executeSequence() {
  // Posición inicial
  int initialAngles[] = {90, 0, 0, 2, 80};
  for (int i = 0; i < 5; i++) {
    moveServo(i, initialAngles[i]);
  }
  delay(1000);

  // Mover brazo a la posición del objeto
  int objectAngles[] = {90, 45, 45, 45, 0};
  for (int i = 0; i < 5; i++) {
    moveServo(i, objectAngles[i]);
  }
  delay(1000);

  // Mover brazo a la posición de destino
  int destinationAngles[] = {150, 20, 20, 20, 80};
  for (int i = 0; i < 5; i++) {
    moveServo(i, destinationAngles[i]);
  }
  delay(1000);

  // Regresar a la posición inicial
  for (int i = 0; i < 5; i++) {
    moveServo(i, initialAngles[i]);
  }
}