package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Pose_subscriber {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "PoseSubscriber" }, new PoseSubscriber());
    }
}

class PoseSubscriber extends PApplet {
    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        RCLP9 rclp9 = new RCLP9(this, "rclp9_Pose_subscriber");
        rclp9.createSubscriber(geometry_msgs.msg.Pose.class, "pose_test_topic", this::subscriberCallback);
        System.out.println("SUBSCRIBER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw(){
        noLoop();
    }

    void subscriberCallback(final geometry_msgs.msg.Pose msg) {
        System.out.println("Received: " + msg);

        // once received a message, exit the program
        exit();
    }
}