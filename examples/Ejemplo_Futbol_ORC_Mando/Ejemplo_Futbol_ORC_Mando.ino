int joy1x;
int joy1y;
int joy2x;
int joy2y;
int joy2pulsador;
void setup() {
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);

  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);

  Serial.begin(57600);
}


void loop() {

  joy1x = analogRead(A5);
  joy1y = analogRead(A4);
  joy2x = analogRead(A3);
  joy2y = analogRead(A2);
  joy2pulsador = analogRead(A6);

  Serial.print("ORC");
  Serial.print(joy1x);
  Serial.print(",");
  Serial.print(joy1y);
  Serial.print(",");
  Serial.print(joy2x);
  Serial.print(",");
  Serial.print(joy2y);
  Serial.print(",");
  if (joy2pulsador < 512) {
    Serial.print(1);
  } else {
    Serial.print(0);
  }
  Serial.println();
}
