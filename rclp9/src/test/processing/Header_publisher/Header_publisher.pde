import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Header_publisher");

void setup() {
    rclp9.createPublisher(std_msgs.msg.Header.class, "test_Header_topic");
    rclp9.createWallClockTimer(500, this::timerCallback);
    rclp9.spin();
}

void timerCallback() {
    rclp9.rcljava.time.Clock clock = new rclp9.rcljava.time.Clock();
    std_msgs.msg.Header message = new std_msgs.msg.Header();
    message.stamp = clock.now();
    message.frame_id = Integer.toString(this.counter);
    this.counter++;
    System.out.println("Publishing: [" + message + "]");
    rclp9.publish(message);
}


void draw() {
    noLoop();
}
