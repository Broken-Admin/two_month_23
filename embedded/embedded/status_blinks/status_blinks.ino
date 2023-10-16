#define GREEN_LED 2
#define RED_LED 3
#define BLUE_LED 4


void setup() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
}

void loop() {
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(BLUE_LED, HIGH);
  delay(1000);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  delay(500);
}
