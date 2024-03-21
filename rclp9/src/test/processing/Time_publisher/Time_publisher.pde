import rclp9.*;

int counter = 0;
RCLP9 rcl = new RCLP9(this, "test_node_rclp9_Time_publisher");

void setup() {
    rcl.createPublisher(builtin_interfaces.msg.Time.class, "test_Time_topic");
    rcl.createWallClockTimer(500, this::timerCallback);
    rcl.spin();
}

void timerCallback() {
    builtin_interfaces.msg.Time message = new builtin_interfaces.msg.Time();
    message = rclp9.rcljava.time.Clock.now();
    System.out.println("Publishing: [" + message + "]");
    rcl.publish(message);
}


void draw() {
    noLoop();
}
