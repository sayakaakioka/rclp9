import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_time_subscriber");
    rclp9.createSubscriber(builtin_interfaces.msg.Time.class, "test_Time_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final builtin_interfaces.msg.Time msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}