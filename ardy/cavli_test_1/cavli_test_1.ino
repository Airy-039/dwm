
#define RST_PIL 12
#define PWRKEY 2  // cavli module Activation action

void setup() {
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(550);
  digitalWrite(PWRKEY, HIGH);

  pinMode(RST_PIL, OUTPUT);
  digitalWrite(RST_PIL, LOW);
  delay(150);
  digitalWrite(RST_PIL, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 32, 33);  // RX, TX
  delay(5000);

  // Serial.println("Sending ATI command to Cavli module");
  // Serial.println("==================================");
  // Serial1.print("ATI");  // Send command via Serial2
  // delay(1000);
  // while (Serial1.available()) {
  //   char c = Serial1.read();
  //   Serial.print(c);
  // }
}

void loop() {


}
