package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Pose_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "PosePublisher" }, new PosePublisher());
    }
}

class PosePublisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_Pose_publisher");

    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        rclp9.createPublisher(geometry_msgs.msg.Pose.class, "pose_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw() {
        noLoop();
    }

    void timerCallback() {
        geometry_msgs.msg.Pose message = new geometry_msgs.msg.Pose();
        double val = (double)1.0/100000000;
        message.position.x = val;
        message.position.y = 2.0*val;
        message.position.z = 4.0*val;
        message.orientation.x = 8.0*val;
        message.orientation.y = 16.0*val;
        message.orientation.z = 32.0*val;
        message.orientation.w = 64.0*val;
        System.out.println("Publishing: [" + message + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }
}