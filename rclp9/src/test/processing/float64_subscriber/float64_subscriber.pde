import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_float64_subscriber");
    rclp9.createSubscriber(std_msgs.msg.Float64.class, "test_float64_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final std_msgs.msg.Float64 msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}