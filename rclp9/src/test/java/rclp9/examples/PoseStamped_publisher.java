package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;
import rclp9.rcljava.time.Clock;

public class PoseStamped_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "PoseStampedPublisher" }, new PoseStampedPublisher());
    }
}

class PoseStampedPublisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "rclp9_PoseStamped_test_publisher");

    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        rclp9.createPublisher(geometry_msgs.msg.PoseStamped.class, "posestamped_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw() {
        noLoop();
    }


    void timerCallback() {
        geometry_msgs.msg.PoseStamped message = new geometry_msgs.msg.PoseStamped();

        message.header.stamp = Clock.now();
        message.header.frame_id = Integer.toString(1);

        double val = (double) 1.0 / 100000000;
        message.pose.position.x = val;
        message.pose.position.y = 2.0 * val;
        message.pose.position.z = 4.0 * val;
        message.pose.orientation.x = 8.0 * val;
        message.pose.orientation.y = 16.0 * val;
        message.pose.orientation.z = 32.0 * val;
        message.pose.orientation.w = 64.0 * val;

        System.out.println("Publishing: [" + message + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }
}
