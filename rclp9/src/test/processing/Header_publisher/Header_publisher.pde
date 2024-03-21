import rclp9.*;

int counter = 0;
RCLP9 rcl = new RCLP9(this, "test_node_rclp9_Header_publisher");

void setup() {
    rcl.createPublisher(std_msgs.msg.Header.class, "test_Header_topic");
    rcl.createWallClockTimer(500, this::timerCallback);
    rcl.spin();
}

void timerCallback() {
    std_msgs.msg.Header message = new std_msgs.msg.Header();
    message.stamp = rclp9.rcljava.time.Clock.now();
    message.frame_id = Integer.toString(this.counter);
    this.counter++;
    System.out.println("Publishing: [" + message + "]");
    rcl.publish(message);
}


void draw() {
    noLoop();
}
