import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_vector3_publisher");

void setup() {
  rclp9.createPublisher(geometry_msgs.msg.Vector3.class, "test_vector3_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  geometry_msgs.msg.Vector3 message = new geometry_msgs.msg.Vector3();
  double val = (double)this.counter/100000000;
  message.x = val;
  message.y = 2.0*val;
  message.z = 4.0*val;
  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
