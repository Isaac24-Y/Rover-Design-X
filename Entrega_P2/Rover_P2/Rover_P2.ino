#include <Servo.h>

Servo servoMotor;
int pos = 90;
String comando;

const int pinTemp = A0;
const int pinLuz = A1;
const int motor_A = 5;
const int motor_B = 6;

void setup() {
  servoMotor.attach(9);
  servoMotor.write(pos);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  Serial.begin(9600);
  Serial.println("Listo para recibir comandos.");
}

void loop() {
  if (Serial.available() > 0) {
    comando = Serial.readStringUntil('\n');
    comando.trim();

    // servo:NN
    if (comando.startsWith("servo:")) {
      String val = comando.substring(6);
      int angle = val.toInt();
      if (angle < 0) angle = 0;
      if (angle > 180) angle = 180;
      servoMotor.write(angle);
      Serial.print("Servo a ");
      Serial.print(angle);
      Serial.println(" deg");
    }
    else if (comando == "temperatura") {
      int lectura = analogRead(pinTemp);
      float temperatura = lectura * 5.0 / 1023.0 * 100.0; // LM35 aproximado
      Serial.print(temperatura, 2);
      Serial.println(" °C");
    }
    else if (comando == "luz") {
      int lectura = analogRead(pinLuz);
      float luz = lectura * 5.0 / 1023.0;
      Serial.print(luz, 2);
      Serial.println(" V");
    }
    else if (comando == "dc:left") {
      analogWrite(motor_A,                                                                                                                                                                                                     );
      digitalWrite(motor_B, 0);
      Serial.println("Motor girando a la izquierda.");
    }
    else if (comando == "dc:right") {
      analogWrite(motor_B, 255);
      digitalWrite(motor_A, 0);
      Serial.println("Motor girando a la derecha.");
    }
    else if (comando == "dc:stop" || comando == "stop") {
      analogWrite(motor_A, 0);
      analogWrite(motor_B, 0);
      Serial.println("Motores detenidos.");
    }
    else {
      Serial.println("Comando inválido.");
    }

    delay(50);
  }
}
