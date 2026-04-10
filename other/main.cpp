#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO055.h>
#include <micro_ros_arduino.h>

// --- Hardware Objects ---
SFE_UBLOX_GNSS_SERIAL myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- Function Prototypes ---
void setup_moving_base_UART();

void setup() {
  // 1. Start Debug Serial (to your PC)
  Serial.begin(115200);
  
  // WAIT for you to open the Serial Monitor (Max 5 seconds)
  while (!Serial && millis() < 5000); 
  Serial.println("\n--- Stern GNSS Initialization Start ---");

  // 2. Start Hardware Serial (to GPS) - Try common baud rates
  long bauds[] = {115200, 38400, 9600};
  bool connected = false;

  for (int i = 0; i < 3; i++) {
    Serial.printf("Attempting connection at %ld baud...\n", bauds[i]);
    Serial1.begin(bauds[i]);
    if (myGNSS.begin(Serial1)) {
      connected = true;
      Serial.printf("Connected successfully at %ld baud!\n", bauds[i]);
      break;
    }
    delay(100);
  }

  if (!connected) {
    Serial.println("GNSS not detected. Please check RX1/TX1 wiring.");
    while (1); // Freeze if hardware isn't found
  }

  // 3. Configure the Moving Base
  setup_moving_base_UART();

  // 4. Initialize IMU
  if (bno.begin()) {
    Serial.println("BNO055 IMU Initialized.");
  } else {
    Serial.println("BNO055 not detected.");
  }

  Serial.println("--- Setup Complete: Stern is now a Moving Base ---");
}

void loop() {
  // Processes incoming UBX/RTCM data from the GPS
  myGNSS.checkUblox();
  
  // Future: Add micro-ROS spin/publish logic here
}

void setup_moving_base_UART() {
  Serial.println("Configuring Moving Base...");

  // 1. First, set the frequency and model (Simple commands)
  myGNSS.setNavigationFrequency(2); 
  myGNSS.setDynamicModel(DYN_MODEL_SEA);

  // 2. Enable RTCM Protocols individually (Avoids VALSET overflow)
  // These use the v3 'set' methods which are often more reliable than bulk VALSET
  bool success = true;
  success &= myGNSS.newCfgValset();
  success &= myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_RTCM3X, 1);
  success &= myGNSS.addCfgValset(UBLOX_CFG_UART1INPROT_RTCM3X, 1);
  myGNSS.sendCfgValset(); // Apply this small batch first

  // 3. Enable the specific RTCM Messages
  myGNSS.newCfgValset();
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 1); 
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 1); 

  if (myGNSS.sendCfgValset()) {
    Serial.println("RTCM Messages enabled successfully!");
  } else {
    Serial.println("RTCM Message configuration FAILED.");
  }

  // 4. LAST STEP: Change the Baud Rate
  // We do this last because as soon as this is applied, 
  // the Teensy and GPS will lose the ability to talk until Serial1 is restarted.
  myGNSS.newCfgValset();
  myGNSS.addCfgValset(UBLOX_CFG_UART1_BAUDRATE, 115200);
  
  if (myGNSS.sendCfgValset()) {
    Serial.println("Baud rate changed to 115200. Restarting Serial1...");
    Serial1.flush();
    Serial1.begin(115200);
    delay(100);
  }

  myGNSS.saveConfiguration(); // Save everything to Flash
}