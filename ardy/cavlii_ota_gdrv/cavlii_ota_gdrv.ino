/*
@INTENTION: OTA using gdrive + cavlii @C16QS
*/
#include "cavliiOTA_.h"

void setup() {
  ini_cavlii();
}

void loop() {
#ifdef SER2cavly_manual
  Ser2cavly_();
#endif
  get_code_();
}
