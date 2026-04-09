#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <Wire.h>

#define LED_PIN 13

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>
#include <std_msgs/msg/string.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO055.h>

// --- SMART ERROR BLINKER ---
void error_loop(int blink_code){
  while(1){
    for(int i=0; i<blink_code; i++){
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
    delay(2000);
  }
}

#define RCCHECK(fn, code) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(code);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- ROS ENTITIES ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t pub_gps;
sensor_msgs__msg__NavSatFix msg_gps;
rcl_publisher_t pub_vel;
geometry_msgs__msg__TwistWithCovarianceStamped msg_vel;
rcl_publisher_t pub_imu;
sensor_msgs__msg__Imu msg_imu; 
rcl_publisher_t pub_imu_status;        
std_msgs__msg__String msg_imu_status;  
char status_buffer[64];                
rcl_timer_t timer_imu;

// --- HARDWARE ---
SFE_UBLOX_GNSS myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// --- TIME SYNC FUNCTION ---
int64_t get_ros_time() {
    if (rmw_uros_epoch_synchronized()) {
        return rmw_uros_epoch_nanos();
    }
    return (int64_t)millis() * 1000000; 
}

// --- IMU CALLBACK (20Hz) ---
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  if (timer == NULL) return;
  
  uint8_t s, g, a, m;
  bno.getCalibration(&s, &g, &a, &m);

  int64_t ros_time = get_ros_time();
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  msg_imu.header.stamp.sec = ros_time / 1000000000;
  msg_imu.header.stamp.nanosec = ros_time % 1000000000;
  
  // Frame ID matches Stern positioning
  msg_imu.header.frame_id.data = (char*)"imu_stern";
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);

  msg_imu.orientation.x = quat.x();
  msg_imu.orientation.y = quat.y();
  msg_imu.orientation.z = quat.z();
  msg_imu.orientation.w = quat.w();
  
  msg_imu.angular_velocity.x = gyro.x();
  msg_imu.angular_velocity.y = gyro.y();
  msg_imu.angular_velocity.z = gyro.z();
  
  msg_imu.linear_acceleration.x = accel.x();
  msg_imu.linear_acceleration.y = accel.y();
  msg_imu.linear_acceleration.z = accel.z();

  float cov = (s >= 3) ? 0.01 : (s == 2) ? 0.1 : 10.0;
  for(int i=0; i<9; i++) {
    msg_imu.orientation_covariance[i] = (i==0 || i==4 || i==8) ? cov : 0.0;
  }

  snprintf(status_buffer, sizeof(status_buffer), "STERN SYS:%d G:%d A:%d M:%d", s, g, a, m);
  msg_imu_status.data.data = status_buffer;
  msg_imu_status.data.size = strlen(msg_imu_status.data.data);

  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
  RCSOFTCHECK(rcl_publish(&pub_imu_status, &msg_imu_status, NULL));
}

// --- GPS PUBLISH ---
void publish_gps_event() {
  if (myGNSS.getPVT()) {
    msg_gps.header.frame_id.data = (char*)"gps_stern"; 
    msg_vel.header.frame_id.data = (char*)"gps_stern"; 
    
    msg_gps.header.stamp.sec = myGNSS.getUnixEpoch();
    msg_gps.header.stamp.nanosec = myGNSS.getNanosecond();
    msg_vel.header.stamp = msg_gps.header.stamp;

    float hAcc_m = (float)myGNSS.getHorizontalAccuracy() / 1000.0f;
    float pos_var = hAcc_m * hAcc_m; 
    if (pos_var < 0.01f) pos_var = 0.01f; 

    msg_gps.position_covariance[0] = pos_var;
    msg_gps.position_covariance[4] = pos_var;
    msg_gps.position_covariance[8] = pos_var * 4.0f;

    float ground_speed = (float)myGNSS.getGroundSpeed() / 1000.0f;
    if (ground_speed < 0.08f) { 
      ground_speed = 0.0f; 
      msg_vel.twist.covariance[0] = 0.001f;
    } else {
      msg_vel.twist.covariance[0] = 0.05f;
    }

    msg_vel.twist.twist.linear.x = ground_speed;
    msg_vel.twist.twist.linear.y = 0.0f;

    msg_gps.latitude = (double)myGNSS.getLatitude() / 10000000.0;
    msg_gps.longitude = (double)myGNSS.getLongitude() / 10000000.0;
    msg_gps.altitude = (double)myGNSS.getAltitudeMSL() / 1000.0;

    RCSOFTCHECK(rcl_publish(&pub_gps, &msg_gps, NULL));
    RCSOFTCHECK(rcl_publish(&pub_vel, &msg_vel, NULL));
  }
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(50);
  }
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator), 1);
  
  // Node name changed to stern
  RCCHECK(rclc_node_init_default(&node, "localizationStern", "", &support), 2);

  while (!rmw_uros_epoch_synchronized()) {
      rmw_uros_sync_session(100);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
  }

  Wire.begin();
  Wire.setClock(400000); // High speed I2C for 10Hz updates

  RCCHECK(rclc_publisher_init_default(&pub_gps, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps_stern/fix"), 3);
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_stern/data"), 4);
  RCCHECK(rclc_publisher_init_default(&pub_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped), "gps_stern/velocity"), 5);
  RCCHECK(rclc_publisher_init_default(&pub_imu_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "imu_stern/status"), 6);

  // IMU Timer at 20Hz (50ms)
  RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), imu_timer_callback), 7);
  
  // INCREASED EXECUTOR HANDLES TO 3 (matches Bow logic for stability)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator), 8);
  RCCHECK(rclc_executor_add_timer(&executor, &timer_imu), 9);

  if (myGNSS.begin()) {
    myGNSS.setI2COutput(COM_TYPE_UBX);
    // BUMPED TO 10HZ (matches Bow)
    myGNSS.setNavigationRate(10); 
    myGNSS.setAutoPVT(true);
  }

  if (bno.begin()) {
    // Keep your unique Stern offsets
    adafruit_bno055_offsets_t stern_offsets = {-34, -51, -39, -2, 1, 0, 579, 178, -677, 1000, 564};
    bno.setSensorOffsets(stern_offsets);
    bno.setExtCrystalUse(true);
  }

  digitalWrite(LED_PIN, HIGH); 
}

void loop() {
  publish_gps_event(); 
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}