package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Twist_subscriber {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "TwistSubscriber" }, new TwistSubscriber());
    }
}

class TwistSubscriber extends PApplet {
    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        RCLP9 rclp9 = new RCLP9(this, "rclp9_Twist_test_subscriber");
        rclp9.createSubscriber(geometry_msgs.msg.Twist.class, "twist_test_topic", this::subscriberCallback);
        System.out.println("SUBSCRIBER READY");
        rclp9.spinOnce();
    }
    
    @Override
    public void draw(){
        noLoop();
    }

    void subscriberCallback(final geometry_msgs.msg.Twist msg) {
        System.out.println("Received: " + msg);

        // once received a message, exit the program
        exit();
    }
}
