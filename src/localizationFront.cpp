#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO055.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/imu.h>

// --- Hardware ---
SFE_UBLOX_GNSS_SERIAL myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- ROS 2 Entities ---
rcl_publisher_t pub_gps, pub_imu;
sensor_msgs__msg__NavSatFix msg_gps;
sensor_msgs__msg__Imu msg_imu;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer_imu;

static char gps_frame[] = "gps_link";
static char imu_frame[] = "imu_link";

const float QUAT_SCALE = 1.0 / (1 << 14);
const float ACCEL_SCALE = 1.0 / 100.0;
const float GYRO_SCALE = 1.0 / 900.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1);}}

// --- GPS Callback ---
void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_gps.header.stamp.sec = time_ns / 1000000000;
  msg_gps.header.stamp.nanosec = time_ns % 1000000000;
  
  msg_gps.latitude = ubxDataStruct->lat / 10000000.0;
  msg_gps.longitude = ubxDataStruct->lon / 10000000.0;
  msg_gps.altitude = ubxDataStruct->hMSL / 1000.0;
  msg_gps.status.status = (ubxDataStruct->fixType >= 3) ? 0 : -1;
  
  rcl_publish(&pub_gps, &msg_gps, NULL);
}

// --- MEGA-BURST I2C READ ---
bool read_imu_mega_burst() {
  uint8_t buf[26];
  Wire.beginTransmission(0x28);
  Wire.write(0x14); 
  if (Wire.endTransmission() != 0) return false;
  
  if (Wire.requestFrom(0x28, 26) != 26) return false;
  for(int i=0; i<26; i++) buf[i] = Wire.read();

  msg_imu.angular_velocity.x = ((int16_t)((buf[1]<<8)|buf[0])) * GYRO_SCALE;
  msg_imu.angular_velocity.y = ((int16_t)((buf[3]<<8)|buf[2])) * GYRO_SCALE;
  msg_imu.angular_velocity.z = ((int16_t)((buf[5]<<8)|buf[4])) * GYRO_SCALE;

  msg_imu.orientation.w = ((int16_t)((buf[13]<<8)|buf[12])) * QUAT_SCALE;
  msg_imu.orientation.x = ((int16_t)((buf[15]<<8)|buf[14])) * QUAT_SCALE;
  msg_imu.orientation.y = ((int16_t)((buf[17]<<8)|buf[16])) * QUAT_SCALE;
  msg_imu.orientation.z = ((int16_t)((buf[19]<<8)|buf[18])) * QUAT_SCALE;

  msg_imu.linear_acceleration.x = ((int16_t)((buf[21]<<8)|buf[20])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.y = ((int16_t)((buf[23]<<8)|buf[22])) * ACCEL_SCALE;
  msg_imu.linear_acceleration.z = ((int16_t)((buf[25]<<8)|buf[24])) * ACCEL_SCALE;

  return true;
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL || !read_imu_mega_burst()) return;

  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_imu.header.stamp.sec = time_ns / 1000000000;
  msg_imu.header.stamp.nanosec = time_ns % 1000000000;

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

void setup() {
  // 1. Hardware Buffers & GPS Startup
  // Adding 1KB buffer to Serial1 to prevent overflows during I2C/ROS operations
  Serial1.addMemoryForRead(new uint8_t[1024], 1024);
  Serial1.begin(115200);
  while(!myGNSS.begin(Serial1)) delay(100);

  myGNSS.setUART1Output(COM_TYPE_UBX); 
  myGNSS.setNavigationFrequency(10); 
  myGNSS.setDynamicModel(DYN_MODEL_SEA);
  myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback); 
  myGNSS.saveConfiguration();

  // 2. IMU Startup & I2C Safety
  Wire.begin();
  Wire.setClock(400000); 
  // Safety timeout: prevent I2C hang from blocking the GPS loop
  Wire.setTimeout(3000); 
  
  while(!bno.begin()) delay(100);
  bno.setExtCrystalUse(true);

  // 3. micro-ROS Setup
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "localizationBowNode", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gps_bow/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_bow/data"));

  msg_gps.header.frame_id.data = gps_frame;
  msg_gps.header.frame_id.size = strlen(gps_frame);
  msg_imu.header.frame_id.data = imu_frame;
  msg_imu.header.frame_id.size = strlen(imu_frame);

  // Timer set to 50Hz (20ms)
  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), imu_timer_callback)); 
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
}

void loop() {
  // Pass 1: Drain GPS before ROS work
  while (myGNSS.checkUblox()) {
    myGNSS.checkCallbacks();
  }

  // ROS work (Strict zero blocking)
  rclc_executor_spin_some(&executor, 0);

  // Pass 2: Catch any bytes that arrived while ROS was publishing
  while (myGNSS.checkUblox()) {
    myGNSS.checkCallbacks();
  }
}