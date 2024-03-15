import rclp9.*;

void setup(){
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Point_subscriber");
    rclp9.createSubscriber(geometry_msgs.msg.Point.class, "test_Point_topic", this::subscriberCallback);
    rclp9.spin();
}

void subscriberCallback(final geometry_msgs.msg.Point msg) {
    System.out.println("Received: " + msg);
}

void draw(){
    noLoop();
}