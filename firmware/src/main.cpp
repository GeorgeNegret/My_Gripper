#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <stdio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h> 


char ssid[] = "TP-LINK_E9E8";
char password[] = "*******";
IPAddress agent_ip(192, 168, 0, 109);
size_t agent_port = 9999;

rcl_subscription_t gripper_subscriber;
std_msgs__msg__Float32 gripper_cmd_msg;

rcl_publisher_t servo_pose_publisher;
std_msgs__msg__Float32 servo_pose_msg;

rcl_publisher_t gripper_joint_state_pub;
sensor_msgs__msg__JointState gripper_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t pub_timer;

Servo gripper_servo;
int gripper_pin = 26;
int pose_open = 0;
int pose_close = 107;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_joint_state_update = 0;

#define SERVO_NUMBERS 1

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    (void) timer;

    RCSOFTCHECK(rmw_uros_sync_session(1000));

    RCSOFTCHECK(rcl_publish(&servo_pose_publisher, &servo_pose_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&gripper_joint_state_pub, &gripper_state_msg, NULL));

}



void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * gripper_cmd_msg = (const std_msgs__msg__Float32 *)msgin; 
}

void move_fingers()
{
  int pose = pose_open;
  if ( gripper_cmd_msg.data > 0.001  ) {
      pose = pose_close;
      gripper_servo.write(pose);
  }    
  else {
      pose = pose_open; 
      gripper_servo.write(pose);
  } 
}

void fill_gripper_state_msg(sensor_msgs__msg__JointState * msg) {
    static double msg_data_tab[3][SERVO_NUMBERS];
    static rosidl_runtime_c__String msg_name_tab[SERVO_NUMBERS];
    char* frame_id = (char*)"gripper_states";
    char* joint_name = (char*)"_finger_left_joint";
    msg->header.frame_id.data = frame_id;
    msg->header.frame_id.capacity = msg->header.frame_id.size = strlen((const char*)frame_id);
    
    msg_name_tab->capacity = msg_name_tab->size = SERVO_NUMBERS;
    msg_name_tab[0].data = (char*)joint_name;
    for(uint8_t i = 0; i < SERVO_NUMBERS; i++){
      msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
    }
    msg->name.capacity = msg->name.size = SERVO_NUMBERS;
    msg->name.data = msg_name_tab;

    if ( gripper_servo.read() > 1 ) {
        static double msg_data_tab[3][SERVO_NUMBERS] = {{0.035}, {0.0}, {0.0}};
        msg->position.data = msg_data_tab[0];
        msg->velocity.data = msg_data_tab[1];
        msg->effort.data   = msg_data_tab[2];
        servo_pose_msg.data = 0.035;
    }
     else {
        static double msg_data_tab[3][SERVO_NUMBERS] = {{0.0}, {0.0}, {0.0}};
        msg->position.data = msg_data_tab[0];
        msg->velocity.data = msg_data_tab[1];
        msg->effort.data   = msg_data_tab[2];
        servo_pose_msg.data = 0.0;
     }
    msg->position.capacity = msg->position.size = SERVO_NUMBERS; 
    msg->velocity.capacity = msg->velocity.size = SERVO_NUMBERS;
    msg->effort.capacity   = msg->effort.size   = SERVO_NUMBERS;

    if (rmw_uros_epoch_synchronized()) 
    {
      msg->header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
      msg->header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
    }
}


void setup() {
  Serial.begin(115200);
 
  gripper_servo.attach(
    gripper_pin,
    Servo::CHANNEL_NOT_ATTACHED, 
    0,
    160
  );

  gripper_servo.write(pose_open);

  set_microros_wifi_transports(ssid, password, agent_ip , agent_port);
  // set_microros_serial_transports(Serial);
  
  Serial.println("ESP32 ip Address: ");
  Serial.println(WiFi.localIP());
  delay(2000);


  // Managementul memoriei pentru micro-ROS
  allocator = rcl_get_default_allocator();
  
  //create init_options - Groups initialization parameters
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "gripper_wifi_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
     &gripper_subscriber,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gripper/cmd") );
   
  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &servo_pose_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gripper/state"));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &gripper_joint_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));

  // create timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback));


  // create executor : Manages the execution of subscriptions and timer callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &gripper_subscriber, &gripper_cmd_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));


  // synchronize time with the agent
  RCCHECK(rmw_uros_sync_session(1000));

  servo_pose_msg.data = gripper_servo.read();
}

void loop() {
  //delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  move_fingers();
  fill_gripper_state_msg(&gripper_state_msg);
}
