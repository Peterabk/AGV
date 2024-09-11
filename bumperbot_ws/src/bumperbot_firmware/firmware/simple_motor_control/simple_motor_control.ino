#define L298N_enA 27
#define L298N_in1 12
#define L298N_in2 13

double cmd = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  //clockwise rotation for motor
  digitWrite(L298N_in1, HIGH);
  digitWrite(L298N_in2, LOW);

  //counter-clockwise rotation for motor
  //digitWrite(L298N_in1, LOW);
  //digitWrite(L298N_in2, HIGH);

  //baud rate
  Serial.begin(115200);
}

void loop() {
  
  if(Serial.available()){

    cmd = Serial.readString().toDouble();
  }

  analogWrite(L298N_enA, cmd*100);

}
