# Dominus-Blanck1
#include <Servo.h>
#include <NewPing.h>
#include <TCS3200.h>

// Pines para controlar el motor con L298
const int motorEnablePin = 5;
const int motorIn1Pin = 6;
const int motorIn2Pin = 7;

// Pines para controlar los servos
const int servo1Pin = 9;
const int servo2Pin = 10;

// Pin para el sensor ultrasónico
const int trigPin = 11;
const int echoPin = 12;

// Pines para el sensor de color
const int S0 = 2;
const int S1 = 3;
const int S2 = 4;
const int sensorOut = A0;

// Duración del movimiento del motor en milisegundos
const unsigned long motorDuration = 8500;   // 8.5 segundos

// Duración del movimiento del servo1 en milisegundos
const unsigned long servo1Duration = 60000; // 1 minuto

// Distancia mínima para detener el motor en centímetros
const int minDistance = 6;

// Duración del movimiento hacia atrás del motor en milisegundos
const unsigned long motorBackwardDuration = 500;  // 0.5 segundos

// Duración del movimiento hacia adelante del motor en milisegundos
const unsigned long motorForwardDuration = 1500;  // 1.5 segundos

// Crear objetos de Servo
Servo servo1;
Servo servo2;

// Crear objeto de NewPing para el sensor ultrasónico
NewPing sonar(trigPin, echoPin);

// Crear objeto de TCS3200 para el sensor de color
TCS3200 colorSensor(S0, S1, S2, sensorOut);

void setup() {
  // Inicializar los pines del motor con L298 como salidas
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);

  // Inicializar los pines de los servos como salidas
  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);

  // Inicializar el pin del sensor ultrasónico como entradas
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Inicializar el sensor de color en modo lectura continua
  colorSensor.begin();
  colorSensor.setMode(TCS3200_MODE_ACTIVE);

  // Inicializar los servos en la posición inicial
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
}

void loop() {
  // Encender el motor durante el tiempo especificado
  startMotor();
  delay(motorDuration);

  // Detener el motor
  stopMotor();

  // Mover el servo1 de izquierda a derecha
  moveServo1LeftToRight();

  // Mover el servo2 de izquierda a derecha durante 1 minuto
  moveServo2LeftToRight();

  // Leer la distancia del sensor ultrasónico
  int distance = readUltrasonicDistance();
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Verificar si se detecta un objeto a una distancia mínima
  if (distance <= minDistance) {
    // Detener el motor
    stopMotor();
    Serial.println("Objeto detectado. Deteniendo el motor.");

    // Leer el color
    int color = readColor();
    Serial.print("Color: ");
    Serial.println(color);

    if (color == TCS3200_COLOR_RED) {
      // Retroceder el motor durante 0.5 segundos
      moveMotorBackward();
      delay(motorBackwardDuration);

      // Avanzar el motor durante 1.5 segundos
      moveMotorForward();
      delay(motorForwardDuration);

      // Mover el servo1 a la izquierda durante 1 segundo
      moveServo1Left();
      delay(1000);
    } else if (color == TCS3200_COLOR_GREEN) {
      // Retroceder el motor durante 0.5 segundos
      moveMotorBackward();
      delay(motorBackwardDuration);

      // Avanzar el motor durante 1.5 segundos
      moveMotorForward();
      delay(motorForwardDuration);

      // Mover el servo1 a la derecha durante 1 segundo
      moveServo1Right();
      delay(1000);
    }
  }
}

void startMotor() {
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, LOW);
  analogWrite(motorEnablePin, 255);
}

void stopMotor() {
  digitalWrite(motorIn1Pin, LOW);
  digitalWrite(motorIn2Pin, LOW);
  analogWrite(motorEnablePin, 0);
}

void moveServo1LeftToRight() {
  for (int angle = 0; angle <= 180; angle++) {
    servo1.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle--) {
    servo1.write(angle);
    delay(15);
  }
}

void moveServo2LeftToRight() {
  unsigned long startTime = millis();
  while (millis() - startTime <= servo1Duration) {
    for (int angle = 0; angle
