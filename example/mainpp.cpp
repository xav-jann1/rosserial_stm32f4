#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

// DMA callbacks:
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

// Publisher:
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

// Subscriber:
void led_cb(const std_msgs::Empty& msg) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
ros::Subscriber<std_msgs::Empty> led_sub("toggle_led", &led_cb);

// Setup node:
void setup(void) {
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(led_sub);
}

// Loop:
void loop(void) {
  // Publish message:
  str_msg.data = hello;
  chatter.publish(&str_msg);

  nh.spinOnce();
  HAL_Delay(500);
}
