import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Header_subscriber");
    rclp9.createSubscriber(std_msgs.msg.Header.class, "test_Header_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final std_msgs.msg.Header msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}