/*
Manufacturer@Cavli Inc.
Model Name@C16QS
Description@LTE CAT1.bis Module
Firmware Release@V1.4.5


# code utlization and need!
its to give seral commant to at command to cavelii<-----
serial monitor--> esp--> serail 1--> cavlii(deafult serial) || for at commands
*/

#define PWRKEY 2  // pg@30 hardware manual
#define RST_PIL 12
#define ESP_GPIO_RX 32
#define ESP_GPIO_TX 33

void setup() {
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
  Serial1.println("ATI");  // Request manufacturer specific information about the TA.(terminal adaptre)
  delay(1000);             // Add a delay after sending the command
}

void loop() {

  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }
  while (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
  }
}

/*
  @AT+COPS? //
    +COPS: 1,2,"405857",7
*/

// Read Command AT+<x>? This command returns the currently set value of
// the parameter or parameters.
// Write Command AT+<x>=<…>
// This command sets the user-definable parameter
// values.
// Execution
// Command
// AT+<x> This command reads non-variable parameters



/*TCP / UDP command flow @35 tcp-udp Aplication notes
AT+CREG? CHECK ANTEENA, WAIT FOR NETWORK CONN..
AT+CGACT? WAIT FOR CONTEXT TO BE SET
AT+HUBBLEREG?

AT+CIPMUX=1 // Multi IP CONFIG
AT+CIPSTART="TCP","docs.google.com",80
AT+CIPSTATUS 
AT+CIPQSEND= 1||0 @This command is used to Select Data Transmitting Mode
AT+CIPSEND=0 // 0 is Connection number ==>Send data without fixed length
AT+CIPSPRT= {MSG}

AT+NETIF? // netwrok info

AT+CIPRXMODE=0// to steram on AT  
AT+TCPFMT=2 // 2 for raw
AT+CIPFLNAME="<filename>",<file_size>

AT+CIPFLINFO="<connection_number>"This is used to displays the size and location of the file

https://drive.google.com/uc?export=download&id=13y5PMM3qBeANLXPanVUgkU_5iSD_NEyo

*/



/////////////////////////////////////////@ HTTP get protocol basic///////////////////////////////////////////////////

/*
============http===============
AT+CREG? ==>Checking network connectivity 
AT+CGACT? ==>Checking internet connectivity @:1,1 means connected
AT+HUBBLEREG? ==>Query Hubble registration status

AT+HTTPURL = {UR URL} // CL1P.NET WORKS WITHOU CERTI.
AT+HTTPADDHEAD= { }
AT+HTTPREQUEST=GET 
AT+HTTPGETHEAD?
AT+HTTPGETSTAT?
AT+HTTPGETCLEN?
AT+HTTPGETCONT?


AT+HTTPCLEAN? ===>: Clear all information of current http(s) session

*************CA CERTI*********** @PG 28 -->HTTP APPLICATION ERV
AT+HTTPSLOAD=1,1  @WRITE CERTI THEN TO EXIST  WRITE >>Hex 1A
*************Write Client certificate
AT+HTTPSLOAD=1,2  @WRITE CERTI
*************: Write Private key
AT+HTTPSLOAD=1,3  @WRITE CERTI

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////to download a file from the google drive//////////////////////////////////////////
// needs auth*! for get request ==> used google api to call

/*@reference & settlements
@ used ==> https://developers.google.com/drive/api/reference/rest/v3
https://developers.google.com/drive/api/reference/rest/v2/files/get

?alt=media // @ usage in link: If you provide the URL parameter alt=media, then the response includes the file contents in the response body. Downloading content with alt=media only works if the file is stored in Drive. To download Google Docs, Sheets, and Slides use files.export instead. For more information, see Download & export files.

-->so to download we need 
GET https://www.googleapis.com/drive/v3/files/{fileId} ||==>> we can get file id  can see in search bar ehen accessed

@then we need to add api key
*=>https://cloud.google.com/docs/authentication/api-keys#console
=>https://cloud.google.com/docs/authentication#authorization
***> response file: https://developers.google.com/drive/api/reference/rest/v3/files#File

@to create project and api key to access the files and content of the drive:
==>https://developers.google.com/workspace/guides/configure-oauth-consent

/*
+++++++++++++DRIVE FILE ACCESS cavlii c16QS
AT+HTTPURL =https://www.googleapis.com/drive/v3/files/13y5PMM3qBeANLXPanVUgkU_5iSD_NEyo?alt=media
AT+HTTPADDHEAD=X-goog-api-key:AIzaSyAXh9sWaDr_TadEeRagRta7tIr9Jlvpi7I
AT+HTTPREQUEST=GET 
AT+HTTPGETHEAD?
AT+HTTPGETSTAT?
AT+HTTPGETCLEN?
AT+HTTPGETCONT?

AT+HTTPCLEAN? ==> IMP TO CLEAR IT OF AS CAN CONFLICT WITH ANOTHER HHTP REQUEST
*/
//1ISPSdIBi2Pg7no8jTYfglyCQZlUDJ-Hc
// 1-u2FYPCXY5-IvacgLHGuLqQUYnUL6y25
// AT+TRB // TO REBOOT
// AT+HTTPADDHEAD=X-goog-api-key:AIzaSyBw7tL1uYLT8hpExEzxOA8QHN3zKdgLQP0
/*

//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
 https://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@raw.githubusercontent.com/Airy-039/dwm/main/Seria1_Test_Terminal.ino.bin


//AT+HTTPURL =https://www.googleapis.com/drive/v3/files/1Dpo3Q4ThHFHUeQJ2MQSykdlzXM2fExmu?alt=media 
// AT+HTTPURL =https://www.googleapis.com/drive/v3/files/1JbhFf8pKQ6sU5px-sMbJ-ZWlcxA2A689?alt=media 
// AT+HTTPADDHEAD=X-goog-api-key:AIzaSyAXh9sWaDr_TadEeRagRta7tIr9Jlvpi7I
// AT+HTTPADDHEAD=Content-Type: application/octet-stream text/plain
// AT+HTTPADDHEAD="Accept: */*"

// AT+HTTPURL =https://www.googleapis.com/drive/v3/files/1qlN2JTGc4isau2nYj5NaLrQp5mISa8Ph?alt=media 
// AT+HTTPADDHEAD=X-goog-api-key:AIzaSyDL5F8pBg9eUumNdiiLDqTcoRKRRWv1kxs
// AT+HTTPREQUEST=GET
// AT+HTTPGETCONT?


AT+HTTPURL =


//to export
// AT+HTTPURL =https://www.googleapis.com/drive/v2/files/1Dpo3Q4ThHFHUeQJ2MQSykdlzXM2fExmu/export?mimeType=application/octet-stream
// mimeType:application/octet-stream
==============how to get file id============
get shareable link.... select the share with everyone who has link
copy link : https://drive.google.com/file/d/1ISPSdIBi2Pg7no8jTYfglyCQZlUDJ-Hc/view?usp=sharing
            https://drive.google.com/file/d/||{=============id=========}||

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////check networks and check humble conn.////////////////////////////////////////////////////
/*
============hubble reg===============
AT^SIMSWAP=? // set to 0 for esim
AT+CREG? //It denotes if the network has been established between the device and the tower @ : 0,1 or 0,5 FOR SUCCESFUL
AT+COPS? //It helps the user to understand which operator network the device has been registered to
AT+CGPADDR // ip address which internet is connected // cid, ip


@hubble registration!!!
AT+CGDCONT? //To set or read the PDP context parameters for each all-local context IDs<cid>
AT+CGACT? //The set command is used to activate or deactivate the specified PDP context
AT+HUBBLEREG=? // WRITE ACCORDING TO THIS IDS AND DATA AND SET IT
AT+HUBBLEREG=<account_id>,<plan_id>[,<group_id>]
AT+HUBBLEREG? //CHECK THE REGISTRATION

AT+TRB // TO REBOOT
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

