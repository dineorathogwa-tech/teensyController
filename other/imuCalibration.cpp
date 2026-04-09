#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/quaternion.h>

// --- Hardware Setup ---
// Using default address 0x28. Change to 0x29 if your jumper is set differently.
extern Adafruit_BNO055 bno; // This says "The object exists elsewhere, just use it."
#define LED_PIN 13

// Software Offsets
float off_ax = 0, off_ay = 0;
unsigned long lastUpdate = 0;
const unsigned int updateInterval = 200; 

void calibration_setup() {
  Serial.begin(115200);
  while(!Serial); 
  pinMode(LED_PIN, OUTPUT);

  if(!bno.begin()) {
    while(1) { // Fast blink on hardware fail
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(50);
    }
  }

  // ==========================================================
  // HARDCODE SECTION: Set to 0 as requested
  // ==========================================================
  adafruit_bno055_offsets_t hardwareOffsets;

  hardwareOffsets.accel_offset_x = -41;
  hardwareOffsets.accel_offset_y = -38;
  hardwareOffsets.accel_offset_z = -23;
  hardwareOffsets.gyro_offset_x  = -4;
  hardwareOffsets.gyro_offset_y  = -1;
  hardwareOffsets.gyro_offset_z  = -2;
  hardwareOffsets.mag_offset_x   = -668;
  hardwareOffsets.mag_offset_y   = -233;
  hardwareOffsets.mag_offset_z   = -754;
  hardwareOffsets.accel_radius   = 1000;
  hardwareOffsets.mag_radius     = 739;


  bno.setSensorOffsets(hardwareOffsets);
  bno.setExtCrystalUse(true);

  // ==========================================================
  // CALIBRATION GATEKEEPER
  // ==========================================================
  Serial.println("\n--- WAITING FOR STABLE CALIBRATION (Target S >= 2) ---");
  uint8_t s, g, a, m;
  
  while (true) {
    bno.getCalibration(&s, &g, &a, &m);
    
    // Live Status Report during wait
    Serial.print("IMU: [S:"); Serial.print(s); 
    Serial.print(" G:"); Serial.print(g); 
    Serial.print(" A:"); Serial.print(a); 
    Serial.print(" M:"); Serial.print(m); 
    Serial.println("]");

    if (s >= 2) {
      Serial.println(">>> CALIBRATION OK! FINALIZING ZEROING... <<<");
      digitalWrite(LED_PIN, HIGH);
      break; 
    }

    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink while waiting
    delay(500); 
  }

  // Initial Zeroing Routine (Software Taring)
  for(int i=0; i<100; i++) {
    imu::Vector<3> v_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    off_ax += v_acc.x(); 
    off_ay += v_acc.y();
    delay(10);
  }
  off_ax /= 100.0; 
  off_ay /= 100.0;
}

void calibration_loop() {
  // 1. READ CALIBRATION
  uint8_t sys, gyr, acc, mag;
  bno.getCalibration(&sys, &gyr, &acc, &mag);

  // LED Logic: Solid only when system is fully fused (3)
  digitalWrite(LED_PIN, (sys == 3));

  // 2. OFFSET PRINT COMMAND
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      adafruit_bno055_offsets_t o;
      bno.getSensorOffsets(o);
      Serial.println("\n======= COPY-PASTE THIS INTO SETUP() =======");
      Serial.println("// IMU Offsets");
      Serial.print("hardwareOffsets.accel_offset_x = "); Serial.print(o.accel_offset_x); Serial.println(";");
      Serial.print("hardwareOffsets.accel_offset_y = "); Serial.print(o.accel_offset_y); Serial.println(";");
      Serial.print("hardwareOffsets.accel_offset_z = "); Serial.print(o.accel_offset_z); Serial.println(";");
      Serial.print("hardwareOffsets.gyro_offset_x  = "); Serial.print(o.gyro_offset_x); Serial.println(";");
      Serial.print("hardwareOffsets.gyro_offset_y  = "); Serial.print(o.gyro_offset_y); Serial.println(";");
      Serial.print("hardwareOffsets.gyro_offset_z  = "); Serial.print(o.gyro_offset_z); Serial.println(";");
      Serial.print("hardwareOffsets.mag_offset_x   = "); Serial.print(o.mag_offset_x); Serial.println(";");
      Serial.print("hardwareOffsets.mag_offset_y   = "); Serial.print(o.mag_offset_y); Serial.println(";");
      Serial.print("hardwareOffsets.mag_offset_z   = "); Serial.print(o.mag_offset_z); Serial.println(";");
      Serial.print("hardwareOffsets.accel_radius   = "); Serial.print(o.accel_radius); Serial.println(";");
      Serial.print("hardwareOffsets.mag_radius     = "); Serial.print(o.mag_radius); Serial.println(";");
      Serial.println("============================================\n");
    }
  }

  // 3. TELEMETRY STREAM
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    imu::Quaternion q = bno.getQuat();
    imu::Vector<3> v_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> v_gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // IMU Print (S, G, A, M included)
    Serial.print("IMU [S"); Serial.print(sys); 
    Serial.print(" G"); Serial.print(gyr); 
    Serial.print(" A"); Serial.print(acc); 
    Serial.print(" M"); Serial.print(mag);
    Serial.print("] Q:"); Serial.print(q.w(),2); Serial.print(","); Serial.print(q.x(),2); Serial.print(","); Serial.print(q.y(),2); Serial.print(","); Serial.print(q.z(),2);
    Serial.print(" A:"); Serial.print(v_acc.x()-off_ax,2); Serial.print(","); Serial.print(v_acc.y()-off_ay,2); Serial.print(","); Serial.print(v_acc.z(),2);
    Serial.print(" G:"); Serial.print(v_gyr.x(),2); Serial.print(","); Serial.print(v_gyr.y(),2); Serial.print(","); Serial.println(v_gyr.z(),2);
  }
}
