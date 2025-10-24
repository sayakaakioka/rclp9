import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_pose_publisher");

void setup() {
  rclp9.createPublisher(geometry_msgs.msg.Pose.class, "test_pose_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  geometry_msgs.msg.Pose message = new geometry_msgs.msg.Pose();
  double val = (double)this.counter/100000000;
  message.position.x = val;
  message.position.y = 2.0*val;
  message.position.z = 4.0*val;
  message.orientation.x = 8.0*val;
  message.orientation.y = 16.0*val;
  message.orientation.z = 32.0*val;
  message.orientation.w = 64.0*val;
  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
