const int motor_A = 5;
const int motor_B = 6;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motor_A, LOW);
  for(int i = 0; i < 255; i++){
    analogWrite(motor_B, i);
  }

}
