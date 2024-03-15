import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Quaternion_subscriber");
    rclp9.createSubscriber(geometry_msgs.msg.Quaternion.class, "test_Quaternion_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final geometry_msgs.msg.Quaternion msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}