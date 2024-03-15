import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Pose_subscriber");
    rclp9.createSubscriber(geometry_msgs.msg.Pose.class, "test_Pose_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final geometry_msgs.msg.Pose msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}