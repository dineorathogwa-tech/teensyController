#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>


SFE_UBLOX_GNSS_SERIAL myGNSS; // Use _SERIAL variant for UART


void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
 Serial.print("Sats: ");  Serial.print(ubxDataStruct->numSV);
 Serial.print("  Lat: "); Serial.print(ubxDataStruct->lat / 10000000.0, 7);
 Serial.print("  Lon: "); Serial.print(ubxDataStruct->lon / 10000000.0, 7);
 Serial.print("  Alt: "); Serial.print(ubxDataStruct->hMSL / 1000.0, 3);
 Serial.println(" m");
}


void setup() {
 Serial.begin(115200);
 delay(2000);


 Serial1.begin(38400);


 if (myGNSS.begin(Serial1) == false) {
   Serial.println("ZED-F9P not detected. Check wiring!");
   while (1);
 }


 myGNSS.setUART1Output(COM_TYPE_UBX);
 myGNSS.setNavigationFrequency(5);
 myGNSS.setAutoPVTcallbackPtr(&pvtCallback); // <-- correct method name in v3
 myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);


 Serial.println("Ready.");
}


void loop() {
 myGNSS.checkUblox();
 myGNSS.checkCallbacks();
}
