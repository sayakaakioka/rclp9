import rclp9.*;

int counter = 0;
RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Time_publisher");

void setup() {
    rclp9.createPublisher(builtin_interfaces.msg.Time.class, "test_Time_topic");
    rclp9.createWallClockTimer(500, this::timerCallback);
    rclp9.spin();
}

void timerCallback() {
    rclp9.rcljava.time.Clock clock = new rclp9.rcljava.time.Clock();
    builtin_interfaces.msg.Time message = new builtin_interfaces.msg.Time();
    message = clock.now();
    System.out.println("Publishing: [" + message + "]");
    rclp9.publish(message);
}


void draw() {
    noLoop();
}
