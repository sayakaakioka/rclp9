import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_twist_publisher");

void setup() {
  rclp9.createPublisher(geometry_msgs.msg.Twist.class, "test_twist_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  geometry_msgs.msg.Twist message = new geometry_msgs.msg.Twist();
  double val = (double)this.counter/100000000;
  message.linear.x = val;
  message.linear.y = 2.0*val;
  message.linear.z = 4.0*val;
  message.angular.x = 8.0*val;
  message.angular.y = 16.0*val;
  message.angular.z = 32.0*val;
  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
