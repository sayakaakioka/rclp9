import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_PoseStamped_subscriber");
    rclp9.createSubscriber(geometry_msgs.msg.PoseStamped.class, "test_PoseStamped_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final geometry_msgs.msg.PoseStamped msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}