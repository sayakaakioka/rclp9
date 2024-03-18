import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_PoseStamped_publisher");

void setup() {
  rclp9.createPublisher(geometry_msgs.msg.PoseStamped.class, "test_PoseStamped_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spin();
}

void timerCallback() {
  rclp9.rcljava.time.Clock clock = new rclp9.rcljava.time.Clock();
  geometry_msgs.msg.PoseStamped message = new geometry_msgs.msg.PoseStamped();

  message.header.stamp = clock.now();
  message.header.frame_id = Integer.toString(this.counter);

  double val = (double) this.counter / 100000000;
  message.pose.position.x = val;
  message.pose.position.y = 2.0 * val;
  message.pose.position.z = 4.0 * val;
  message.pose.orientation.x = 8.0 * val;
  message.pose.orientation.y = 16.0 * val;
  message.pose.orientation.z = 32.0 * val;
  message.pose.orientation.w = 64.0 * val;

  this.counter++;
  System.out.println("Publishing: [" + message + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
