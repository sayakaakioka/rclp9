import rclp9.*;

int counter = 0;
RCLP9 rcl = new RCLP9(this, "test_node_rclp9_posestamped_publisher");

void setup() {
  rcl.createPublisher(geometry_msgs.msg.PoseStamped.class, "test_posestamped_topic");
  rcl.createWallClockTimer(500, this::timerCallback);
  rcl.spin();
}

void timerCallback() {
  geometry_msgs.msg.PoseStamped message = new geometry_msgs.msg.PoseStamped();

  message.header.stamp = rclp9.rcljava.time.Clock.now();
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
  rcl.publish(message);
}


void draw() {
  noLoop();
}
