import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_float64_publisher");

void setup() {
  rclp9.createPublisher(std_msgs.msg.Float64.class, "test_float64_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  std_msgs.msg.Float64 message = new std_msgs.msg.Float64();
  message.data = (double)this.counter/100000000;
  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
