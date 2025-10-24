package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;
import rclp9.rcljava.time.Clock;

public class Header_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "HeaderPublisher" }, new HeaderPublisher());
    }
}

class HeaderPublisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "rclp9_Header_test_publisher");

    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        rclp9.createPublisher(std_msgs.msg.Header.class, "header_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw() {
        noLoop();
    }

    void timerCallback() {
        int counter = 0;
        std_msgs.msg.Header message = new std_msgs.msg.Header();
        message.stamp = Clock.now();
        message.frame_id = Integer.toString(counter);
        System.out.println("Publishing: [" + message + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }

}
