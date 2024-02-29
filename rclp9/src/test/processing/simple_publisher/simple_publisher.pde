import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9");

void setup() {
  rclp9.createPublisher(std_msgs.msg.String.class, "test_topic");
  rclp9.createWallClockTimer(500, this::timerCallback);
  rclp9.spinPublisher();
}

void timerCallback() {
  std_msgs.msg.String message = new std_msgs.msg.String();
  message.setData("Hello, world! " + this.counter);
  this.counter++;
  System.out.println("Publishing: [" + message.getData() + "]");
  rclp9.publish(message);
}


void draw() {
  noLoop();
}
