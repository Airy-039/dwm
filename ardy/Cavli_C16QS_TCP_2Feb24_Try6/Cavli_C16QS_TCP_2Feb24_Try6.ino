// In this test we're using Serial2 for Cavli AT response directly send to Serial of ESP32
/*

  Auth: Raj Mehta
  Last Updates: 16 January 2024

  Manufacturer: Cavli Inc.
  Model Name: C16QS
  Description: LTE CAT1.bis Module
  Firmware Release: V1.4.5
  IMEI: 358773400071148
  Serial Number: QCLC17114Z
  HW Version: C16QS_HW_V2.21(2711)
  Part Number: C16QS-EA-GNAH
  Build Date: 20231215

  //  TCP commands
  AT+COPS?
  AT+CGACT?
  AT+CREG?
  AT+NETIF?
  AT+CIPSTART=TCP,petpooja-app-builds.s3.ap-south-1.amazonaws.com,80
  AT+CIPSEND=4158
  GET /payroll/Payroll_Firmware_v1.0.ino.bin HTTP/1.1
  Host: petpooja-app-builds.s3.ap-south-1.amazonaws.com
  Content-Type: text/plain
  Content-Length: 4000

*/
#define RST_PIL 12
#define PWRKEY 2  // cavli module Activation action
/* Cavli_C16QS */
String response = "";
String st1;
int value1, value2;
int clientID;

void sendChunks(const char *data) {
  const int chunkSize = 110;  // Adjust the chunk size as needed
  int dataLength = strlen(data);

  for (int i = 0; i < dataLength; i += chunkSize) {
    // Send a chunk of data
    Serial1.write(data + i, min(chunkSize, dataLength - i));

    // Add a delay between chunks if needed
    delay(100);
  }
}
void extractValues(String inputString, int &resultValue1, int &resultValue2) {
  // Find the position of the comma
  int commaPosition = inputString.indexOf(',');

  // Check if the comma was found
  if (commaPosition != -1) {
    // Extract the values before and after the comma
    String value1String = inputString.substring(inputString.indexOf(':') + 2, commaPosition);
    String value2String = inputString.substring(commaPosition + 1);

    // Convert the extracted strings to integers
    resultValue1 = value1String.toInt();
    resultValue2 = value2String.toInt();
  } else {
    Serial.println("Comma not found in the input string");
    resultValue1 = -1;  // Return a default value or an error code
    resultValue2 = -1;  // Return a default value or an error code
    // to reset cavli
    Serial.println("Sending AT command... +TRB");
    Serial1.write("AT+TRB\r\n");
    FeedBack();
    delay(4000);
    // to reset esp
    ESP.restart();
  }
}
bool FeedBack() {
  // Wait until "OK" or "ERROR" is found
  response = "";
  while (!(response.indexOf("OK") != -1)) {  // || response.indexOf("ERROR") != -1)) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
    Serial.print("response-->");
    Serial.println(response);
  }
  return 0;
}


void setup() {
  // Power up the ESP32 by toggling the PWRKEY pin
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(100);
  digitalWrite(PWRKEY, HIGH);

  pinMode(RST_PIL, OUTPUT);
  digitalWrite(RST_PIL, LOW);
  delay(100);
  digitalWrite(RST_PIL, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 32, 33);  // RX, TX

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(500);
  digitalWrite(2, HIGH);
  //////////////////////////////////
  delay(5000);
  // Send AT command via Serial to Cavli module
  Serial.println("Sending ATI command to Cavli module");
  Serial.println("==================================");
  Serial1.print("ATI");  // Send command via Serial2
  // FeedBack();
  delay(1000);
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }
  // response = "";
  // Serial.println("Sending AT+QSPN");
  // Serial1.print("AT+QSPN");  // Send command via Serial1
  // // FeedBack();
  // delay(1000);
  // while (Serial1.available()) {
  //   char c = Serial1.read();
  //   Serial.print(c);
  // }
  // response = ""; 

  Serial.println("Sending AT+COPS?");
  //////////////////////////////////
  // Read and print responses from Cavli module via Serial2
  while (!(response.indexOf("+ATREADY") != -1 || response.indexOf("OK") != -1)) {  // || response.indexOf("ERROR") != -1)) {
    Serial1.print("AT+COPS?\r\n");                                                 // Send command via Serial2
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
    // Serial.print("response-->");
    // Serial.println(response);
  }
  delay(1000);  // Add a delay after sending the command
  response = "";
  //////////////////////////////////
  value1 = value2 = -1;
  Serial.println("Sending AT command... +CEREG? Checking network connectivity");
  while (!(value1 == 0 && (value2 == 1 || value2 == 5))) {
    Serial1.write("AT+CEREG?\r\n");
    delay(500);
    FeedBack();
    extractValues(response, value1, value2);
  }
  Serial.print(" response:");
  Serial.println(response);
  delay(5000);  // Add a delay after sending the command
  response = "";
  //////////////////////////////////
  value1 = value2 = -1;
  Serial.println("Sending AT command... +CGACT? Checking internet connectivity");
  while (!(value1 == 1 && value2 == 1)) {
    Serial1.write("AT+CGACT?\r\n");
    delay(500);
    FeedBack();
    extractValues(response, value1, value2);
  }
  Serial.print("Response:");
  Serial.print(response);
  //////////////////////////////////
  delay(500);  // Add a delay after sending the command
  response = "";
  Serial.println("Sending AT+NETIF?");
  Serial1.print("AT+NETIF?\r\n");  // Send command via Serial2
  // Read and print responses from Cavli module via Serial2
  FeedBack();

  //////////////////////////////////
  response = "";
  Serial.println("Sending AT+TCPFMT=2");
  Serial1.print("AT+TCPFMT=2\r\n");  // Send command via Serial2
  // Read and print responses from Cavli module via Serial2
  FeedBack();
  //////////////////////////////////

  //AT+PING
  // Serial.println("Sending AT+PING");
  // Serial1.print("AT+PING=google.com\r\n");  // Send command via Serial2
  // Read and print responses from Cavli module via Serial2
  // FeedBack();
  delay(500);  // Add a delay after sending the command
  response = "";
  Serial.println("Sending AT+CIPSTART=TCP,petpooja-app-builds.s3.ap-south-1.amazonaws.com,80");
  Serial1.print("AT+CIPSTART=TCP,petpooja-app-builds.s3.ap-south-1.amazonaws.com,80\r\n");  // Send command via Serial2
  while (!(response.indexOf("OK") != -1)) {
    // Read and print responses from Cavli module via Serial2
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
    delay(100);

    Serial.print("Response:-------->");
    Serial.print(response);
  }
  delay(1000);
  Serial.println("Sending AT+CIPSTATUS");
  Serial1.print("AT+CIPSTATUS\r\n");  // Send command via Serial2
  delay(1500);
  while (Serial1.available()) {
    char c = Serial1.read();
    response += c;
  }
  Serial.print("Response:-----########--->");
  Serial.print(response);

  Serial.println("Sending AT+CIPSEND=4158");
  response = "";
  Serial1.print("AT+CIPSEND=4158\r\n");  // Send command via Serial2
  // Read and print responses from Cavli module via Serial2
  while (!(response.indexOf(">") != -1)) {  // || response.indexOf("ERROR") != -1)) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }

  Serial.println("Sending GET ....:");
  Serial1.print("GET /payroll/Payroll_Firmware_v1.0.ino.bin HTTP/1.1\r\nHost: petpooja-app-builds.s3.ap-south-1.amazonaws.com\r\nContent-Type: text/plain\r\nContent-Length: 4000\r\n");

  // delay(100);

  Serial.println("Sending ....Data:");
  Serial1.print(F("\r\n{\"help\":\"89075337635954647102216077828334038179638965680796041208684221958292459041499210220101176726750119361506928946801115768179724422446326727588985277987663261869651895734967254175768432363323845153250669679670847292366626594244272038307891788559603898885967160716100064558829358466998466502019977782814896774797052702060810292198055643589924164658242199256903875497344936216080531239955776017464552148435666505635723861800404618684226812752076597749931057757306056294943547000781262839809899923354086591584210040451607204102188927821539179159381632631527501971993045061268210482892916402716683349257472025692410570321628744103831322940664906113645556950358984065099276387006262572153135702022428247137918956448523542982648187122261597220614110947356933150068277012583532229033541176707338889365478924968936854152065761173915111644345900669149625078722916074227614326832708754591535336522843069706165303225712869367519796378513590306575406497476472835442756979658795508703710440546642985922399004612159467600905955755449966018437890191460823649954507322341178320083272336399693848042281362677975607570272200287873519860742931419344002094736767271088986498050654477655967376633678898223733576930739058637767383825961935849401218839710209402921257312961820937048252605134241355821788312444060405165880961083090291348224018185135057406955502613542542029737527538203202790873609471965591842962124131036383648949190295669203778994146624711175606317374371182549375099048319909535463225194873558144354057114785133576705451417485999639036485103175235333968552793775415105863301060242033137339301553612465043007580178633002347254939486774762533517402544695229622956692043054479528248190322688938254889287474299252637990906223917476170475963913210665367126385577323757568086744527395155415304803066559869339386362704532293086384746844676629443284782157107953249054516787232904553940994901593424779192253246902943606375448951225118448301956086168374658315461330349815680774676027301983519593091618714763496693321327527675600113213326246993327253043705895216827844497053039199992678037335735636970855511533789798545930065775480899099817135432807604607183972679352907626052597109494544611563375735532140173762602900790266634254116386656255204733305180421292662233028241096775669616585889040375006715884019811536956240585233400637946794045905653897316207855958856671262695550818746832431295606706526289163496303286926212887168424390792956595788418411214541204134910689891158586305421302002182835946950120614238970904406183421856068983175081423184410264460437909042807028579720246240765801109735402262520334811735897298457687101805703544400362287704020021974124384513179485429737379288056395947178851257079604107224206190139277216709889542925856531989476001916224619027688637593875639728305081627298012519538666788210155468933245923286053294125031545706572752368877157756923406743204680723263550965286209509075800151784323706302506190770996523471352711384798790768453597153988434949643971339330860753816907351650932057319881717677921921624342488023285457464136994664772067954504249230069924114917578758575435078932814946225816723664286840764088429039750934904903682715511723666111536635021192514121284993224325008509407099548302677095306550154527100456899219533445259632518197950620310970426055165021001199607534387773800051156886007543422944697944210111434092120204627469207657616686329145149953087544164706838697712066307637631542308577890521950715395956733727831900957419936036043512013354318852228554517014871430445521830069338532097049797162084341603896805603968173620873488686321825585383337154948477264398298078899648827629100898503758814237454761873103919617993607280903624542298395444434279573824725438545745784080597176335905458320174243596110768697939661062324810103683410495617851410713572620315448726583584981449947038985485575281475744247630223527779072265880742100969654904378021631591548917978843716078859845199246451507198522989869706822421809375498599317693238927267951802056532859066501582\"}"));
  delay(2000);

  // Wait for a while before sending the next data (adjust as needed)

  response = "";
  Serial.println("----------------------------End Setup----------------------------");
}

String input;
void loop() {
  while (Serial1.available()) {
    input = Serial1.readStringUntil('\n');
    response += input;
    if (response.length() > 0) {
      Serial.println(response);
      // if (response.endsWith("CLOSED")) {
      //   Serial.println("response.endsWith(CLOSE)");
      //   break;
      // }
      response = "";
    } 
  }
  response = "";
}
