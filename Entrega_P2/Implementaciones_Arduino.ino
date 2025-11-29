#include <Servo.h>

// =========================
// CONFIGURACIÓN DE PINES
// =========================

// Sensor LDR (divisor de voltaje)
#define LDR_PIN A0

// Sensor de temperatura (LM35)
#define TEMP_PIN A1

// Motor DC con transistor o relevador
#define MOTOR_PIN 9

// Servo
Servo servo1;
#define SERVO1_PIN 10

// Opcional: un segundo servo
Servo servo2;
#define SERVO2_PIN 11


// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  servo1.attach(SERVO1_PIN);
  servo1.write(90);

  servo2.attach(SERVO2_PIN);
  servo2.write(90);

  Serial.println("ARDUINO NANO LISTO");
}


// =========================
// FUNCIONES DE SENSORES
// =========================

// ---- LDR ----
float leerLuz() {
  int raw = analogRead(LDR_PIN);
  float volt = raw * (5.0 / 1023.0);
  return volt;
}

// ---- LM35 ----
float leerTemperatura() {
  int raw = analogRead(TEMP_PIN);
  float volt = raw * (5.0 / 1023.0);
  float tempC = volt * 100.0;  // LM35 => 10mV por °C
  return tempC;
}


// =========================
// LOOP PRINCIPAL
// =========================
void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Debug
    Serial.print("CMD: ");
    Serial.println(cmd);

    // ==========================================
    // SENSORES
    // ==========================================

    if (cmd == "temp") {
      float t = leerTemperatura();
      Serial.println(t);
    }

    else if (cmd == "luz") {
      float luz = leerLuz();
      Serial.println(luz);
    }

    // ==========================================
    // MOTOR DC
    // ==========================================

    else if (cmd == "dc") {
      digitalWrite(MOTOR_PIN, HIGH);
      delay(300);                   // activación breve
      digitalWrite(MOTOR_PIN, LOW);
      Serial.println("DC_OK");
    }

    // ==========================================
    // SERVOS
    // ==========================================

    else if (cmd == "servo1") {
      servo1.write(90);
      Serial.println("SERVO1_OK");
    }

    else if (cmd == "servo2") {
      servo2.write(90);
      Serial.println("SERVO2_OK");
    }

    else if (cmd == "servo_left") {
      servo1.write(0);
      Serial.println("SL");
    }

    else if (cmd == "servo_right") {
      servo1.write(180);
      Serial.println("SR");
    }

    // ==========================================
    // STOP GENERAL
    // ==========================================
    else if (cmd == "stop") {
      digitalWrite(MOTOR_PIN, LOW);
      servo1.write(90);
      servo2.write(90);
      Serial.println("STOP_OK");
    }

    // ==========================================
    // COMANDO NO RECONOCIDO
    // ==========================================
    else {
      Serial.println("ERR");
    }

  }
}
