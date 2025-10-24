package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Header_subscriber {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "HeaderSubscriber" }, new HeaderSubscriber());
    }
}

class HeaderSubscriber extends PApplet {
    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        RCLP9 rclp9 = new RCLP9(this, "rclp9_Header_test_subscriber");
        rclp9.createSubscriber(std_msgs.msg.Header.class, "header_test_topic", this::subscriberCallback);
        System.out.println("SUBSCRIBER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw(){
        noLoop();
    }

    void subscriberCallback(final std_msgs.msg.Header msg) {
        System.out.println("Received: " + msg);

        // once received a message, exit the program
        exit();
    }
    
}
