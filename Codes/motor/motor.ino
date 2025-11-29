const int motor_A = 5;
const int motor_B = 6;
const int motor_C = 10;
const int motor_D = 11;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_C, OUTPUT);
  pinMode(motor_D, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(motor_B, 255);
  analogWrite(motor_C, 255);
  delay(500);
  digitalWrite(motor_A, LOW);
  digitalWrite(motor_D, LOW);
  delay(500);


}
