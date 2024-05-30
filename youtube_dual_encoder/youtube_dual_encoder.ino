#define CSn1 51
#define CSn2 52
#define CSn3 53


int posA=0;
int posB=0;
int posC=0;

void setup() {
  Serial.begin(9600);
  
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CSn1, OUTPUT);
  pinMode(CSn2, OUTPUT);
  pinMode(CSn3, OUTPUT);
  digitalWrite(CSn1, HIGH);
  digitalWrite(CSn2, HIGH);
  digitalWrite(CSn3, HIGH);

}


void loop() {
  posA = readEncoder(CSn1);
  int eposA=map(posA,0,16384,0,359);
  eposA=constrain(eposA,0,359);
  Serial.print("posA: ");
  Serial.print(eposA);

  posB = readEncoder(CSn2);
  int eposB=map(posB,0,16384,0,359);
  eposB=constrain(eposB,0,359);
  Serial.print(", posB: ");
  Serial.print(eposB);

  posC = readEncoder(CSn3);
  int eposC=map(posC,0,16384,0,359);
  eposC=constrain(eposC,0,359);
  Serial.print(", posC: ");
  Serial.println(eposC);

delay(10);
}

int readEncoder(int EncPin) {
  digitalWrite(EncPin, LOW);
  byte d = shiftIn(MISO, SCK, MSBFIRST);
  byte d1 = shiftIn(MISO, SCK, MSBFIRST);
  digitalWrite(EncPin, HIGH);
  d &=0b00111111;
  int posEnc = d << 8;
  posEnc = posEnc | d1;
  return posEnc;
}