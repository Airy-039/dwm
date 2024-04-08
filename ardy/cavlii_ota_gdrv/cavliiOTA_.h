#define CavliiCom
// #define SER2cavly_manual

#define PWRKEY 2  // pg@30 hardware manual
#define RST_PIL 12
#define ESP_GPIO_RX 32
#define ESP_GPIO_TX 33

#ifdef CavliiCom
#define CavliiMsg(Msg_) Serial1.println(Msg_)
#endif

void ini_cavlii() {
  // Power up the ESP32 by toggling the PWRKEY pin
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(550);
  digitalWrite(PWRKEY, HIGH);

  pinMode(RST_PIL, OUTPUT);
  digitalWrite(RST_PIL, LOW);
  delay(150);
  digitalWrite(RST_PIL, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, ESP_GPIO_RX, ESP_GPIO_TX);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(500);
  digitalWrite(2, HIGH);

  // Send AT command via Serial to Cavli module
  Serial.println("Sending ATI command to Cavli module");
  Serial.println("==================================");
  CavliiMsg("ATI");  // Request manufacturer specific information about the TA.(terminal adaptre)
  delay(1000);       // Add a delay after sending the command
}

void PrintResp() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }
}
void Ser2cavly_() {
  PrintResp();
  while (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
  }
}
void check_ntwrk() {
  CavliiMsg("AT+CREG?");
  PrintResp();
  CavliiMsg("AT+CGACT?");
  PrintResp();
  CavliiMsg("AT+HUBBLEREG?");
  PrintResp();
}
void get_code_() {
  check_ntwrk();
  // CavliiMsg("AT+HTTPURL =https://www.googleapis.com/drive/v3/files/1ISPSdIBi2Pg7no8jTYfglyCQZlUDJ-Hc?alt=media");
  // PrintResp();
  // CavliiMsg("AT+HTTPADDHEAD=X-goog-api-key:AIzaSyAXh9sWaDr_TadEeRagRta7tIr9Jlvpi7I");
  // PrintResp();
  // CavliiMsg("AT+HTTPREQUEST=GET");
  // delay(3000);
  // PrintResp();
  // CavliiMsg("AT+HTTPGETSTAT?");
  // PrintResp();
  // delay(100);
  // CavliiMsg("AT+HTTPGETCONT?");
  // PrintResp();
  // delay(100);

  // while(1);
}
