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
rcl_publisher_t pub_gps;
rcl_publisher_t pub_imu;
sensor_msgs__msg__NavSatFix msg_gps;
sensor_msgs__msg__Imu msg_imu;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer_imu;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1);}}

// --- GPS Callback (UART Binary Stream) ---
void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // Use rmw_uros_epoch_nanos() for accurate synced ROS time
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_gps.header.stamp.sec = time_ns / 1000000000;
  msg_gps.header.stamp.nanosec = time_ns % 1000000000;
  
  msg_gps.latitude = ubxDataStruct->lat / 10000000.0;
  msg_gps.longitude = ubxDataStruct->lon / 10000000.0;
  msg_gps.altitude = ubxDataStruct->hMSL / 1000.0;
  msg_gps.status.status = (ubxDataStruct->fixType >= 3) ? 0 : -1;
  msg_gps.header.frame_id.data = (char*)"gps_link";
  msg_gps.header.frame_id.size = strlen(msg_gps.header.frame_id.data);

  rcl_publish(&pub_gps, &msg_gps, NULL);
}

// --- IMU Callback (Timer Driven) ---
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;

  // I2C Reads (The bottleneck)
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_imu.header.stamp.sec = time_ns / 1000000000;
  msg_imu.header.stamp.nanosec = time_ns % 1000000000;
  msg_imu.header.frame_id.data = (char*)"imu_link";
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);

  msg_imu.orientation.x = quat.x();
  msg_imu.orientation.y = quat.y();
  msg_imu.orientation.z = quat.z();
  msg_imu.orientation.w = quat.w();

  float d2r = PI / 180.0;
  msg_imu.angular_velocity.x = gyro.x() * d2r;
  msg_imu.angular_velocity.y = gyro.y() * d2r;
  msg_imu.angular_velocity.z = gyro.z() * d2r;

  msg_imu.linear_acceleration.x = accel.x();
  msg_imu.linear_acceleration.y = accel.y();
  msg_imu.linear_acceleration.z = accel.z();

  rcl_publish(&pub_imu, &msg_imu, NULL);
}

void setup() {
  // 1. GPS UART Connection with Handshake
  Serial1.begin(115200);
  delay(500);
  if (myGNSS.begin(Serial1) == false) {
    Serial1.begin(38400);
    if (myGNSS.begin(Serial1)) {
      myGNSS.setSerialRate(115200);
      delay(100);
      Serial1.begin(115200);
    }
  }

  // 2. Optimization: Kill all NMEA noise
  myGNSS.setUART1Output(COM_PORT_UART1, COM_TYPE_UBX); 
  myGNSS.setNavigationFrequency(10); 
  myGNSS.setDynamicModel(DYN_MODEL_SEA);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback); 
  myGNSS.saveConfiguration();

  // 3. IMU Initialization
  Wire.begin();
  Wire.setClock(400000); 
  if (!bno.begin()) { /* error code */ }

  // 4. Micro-ROS setup
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_usv_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gps/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));

  // Timer set to 50ms (20Hz) - Highest stable speed for this bus combo
  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(50), imu_timer_callback)); 
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
}

void loop() {
  // 1. Drain GPS buffer immediately
  while (myGNSS.checkUblox()) {
    myGNSS.checkCallbacks();
  }
  // 2. Perform ROS tasks
  rclc_executor_spin_some(&executor, 10);
}