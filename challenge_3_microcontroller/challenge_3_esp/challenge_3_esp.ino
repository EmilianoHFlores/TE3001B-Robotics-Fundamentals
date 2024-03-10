#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32 pub_msg;
std_msgs__msg__Float32 sub_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t adc_timer;

#define LED_PIN 25
#define POTENTIOMETER_PIN 33

#define PWM_LED_PIN 32
#define FREQUENCY 5000
#define LED_CHANNEL 0
#define RESOLUTION 8

#define ADC_TIMER_TIMEOUT 10
#define PUBLISHER_TIMER_TIMEOUT 100

const char* node_name = "micro_ros_esp32_node";
const char* publisher_topic = "micro_ros_esp32/raw_pot";
const char* subscriber_topic = "micro_ros_esp32/pwm_duty_cycle";

float potentiometer_voltage = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = potentiometer_voltage;
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void adc_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int potentiometer_raw = analogRead(POTENTIOMETER_PIN);
    potentiometer_voltage = potentiometer_raw * (3.3 / 4095.0);
  }
}

void pwm_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    ledcWrite(LED_CHANNEL, msg->data / 3.3 * 255);
}

void setup() {
    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  
    pinMode(POTENTIOMETER_PIN, INPUT);
    ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(PWM_LED_PIN, LED_CHANNEL);
    ledcWrite(LED_CHANNEL, 0);

    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    publisher_topic));
    // create timer for adc
    RCCHECK(rclc_timer_init_default(
        &adc_timer,
        &support,
        RCL_MS_TO_NS(ADC_TIMER_TIMEOUT),
        adc_timer_callback));
    
    // create timer for publishing
    RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUBLISHER_TIMER_TIMEOUT),
    publisher_timer_callback));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    subscriber_topic));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &adc_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &pwm_subscription_callback, ON_NEW_DATA));

    pub_msg.data = 0;
}

void loop() {
    delay(5);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}