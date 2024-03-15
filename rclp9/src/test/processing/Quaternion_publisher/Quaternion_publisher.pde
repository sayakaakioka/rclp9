import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Quaternion_publisher");

void setup() {
  rclp9.createPublisher(geometry_msgs.msg.Quaternion.class, "test_Quaternion_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  geometry_msgs.msg.Quaternion message = new geometry_msgs.msg.Quaternion();
  double val = (double)this.counter/100000000;
  message.quaternion(val, 2.0*val, 4.0*val, 8.0*val);
  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
