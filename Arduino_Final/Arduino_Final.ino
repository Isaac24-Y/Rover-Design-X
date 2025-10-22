#include <Servo.h>

// Pines
const int LDR_PIN = A0;       // Sensor de luz
const int DC_MOTOR_PIN = 5;   // Motor DC (PWM)
const int SERVO_PIN = 9;      // Servomotor

// Objetos
Servo servoMotor;

// Variables
bool lectura_analogica = false;
bool motor_dc_activo = false;
bool servo_activo = false;

unsigned long ultimo_envio = 0;
const unsigned long intervalo_envio = 500;  // Enviar cada 0.5 segundos

void setup() {
  Serial.begin(9600);
  pinMode(LDR_PIN, INPUT);
  pinMode(DC_MOTOR_PIN, OUTPUT);
  servoMotor.attach(SERVO_PIN);

  Serial.println("Arduino listo ✅");
}

void loop() {
  // ─── Recepción de comandos ───
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "START_ANALOG") {
      lectura_analogica = true;
      Serial.println("ACK_ANALOG_ON");
    } 
    else if (comando == "STOP_ANALOG") {
      lectura_analogica = false;
      Serial.println("ACK_ANALOG_OFF");
    } 
    else if (comando == "START_DC") {
      motor_dc_activo = true;
      Serial.println("ACK_DC_ON");
    } 
    else if (comando == "STOP_DC") {
      motor_dc_activo = false;
      analogWrite(DC_MOTOR_PIN, 0);
      Serial.println("ACK_DC_OFF");
    } 
    else if (comando == "START_SERVO") {
      servo_activo = true;
      Serial.println("ACK_SERVO_ON");
    } 
    else if (comando == "STOP_SERVO") {
      servo_activo = false;
      Serial.println("ACK_SERVO_OFF");
    }
  }

  // ─── Enviar lectura del LDR ───
  if (lectura_analogica && millis() - ultimo_envio > intervalo_envio) {
    ultimo_envio = millis();
    int valor = analogRead(LDR_PIN);
    Serial.print("LDR:");
    Serial.println(valor);
  }

  // ─── Control del motor DC ───
  if (motor_dc_activo) {
    analogWrite(DC_MOTOR_PIN, 180); // Velocidad media
  }

  // ─── Movimiento del servo ───
  if (servo_activo) {
    for (int angulo = 0; angulo <= 180; angulo += 10) {
      servoMotor.write(angulo);
      delay(15);
    }
    for (int angulo = 180; angulo >= 0; angulo -= 10) {
      servoMotor.write(angulo);
      delay(15);
    }
  }
}
