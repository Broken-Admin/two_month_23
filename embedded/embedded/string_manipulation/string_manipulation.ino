String data = "1";
int count = 1;
void setup() {
    Serial.println(data);
}

void loop() {
    delay(1000);
    if(count > 9) {count = 0;}
    count++;
    data += String(count);
    Serial.println(data);
}