#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Bool.h> // Assuming the messages are of type std_msgs/Bool

ros::NodeHandle nh;

#define LED2_PIN GPIO_PIN_5
#define LED2_GPIO_PORT GPIOA

// Define a callback function for the subscriber
void ledControlCallback(const std_msgs::Bool& led_state) {
    if (led_state.data) {
        // Code to turn the LED ON
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET); // Replace GPIOx and GPIO_PIN_y with actual LED pin
    } else {
        // Code to turn the LED OFF
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET); // Replace GPIOx and GPIO_PIN_y with actual LED pin
    }
}

// Initialize the subscriber with the topic name and the callback function
ros::Subscriber<std_msgs::Bool> sub("led_control", &ledControlCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  // Register the subscriber
  nh.subscribe(sub);
}

void loop(void)
{
  nh.spinOnce(); // This will call the ledControlCallback when a message is received

  HAL_Delay(1000);
}


