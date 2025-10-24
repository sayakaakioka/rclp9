package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Time_subscriber {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "TimeSubscriber" }, new TimeSubscriber());
    }
}

class TimeSubscriber extends PApplet {
    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        RCLP9 rclp9 = new RCLP9(this, "test_node_rclp9_time_subscriber");
        rclp9.createSubscriber(builtin_interfaces.msg.Time.class, "time_test_topic", this::subscriberCallback);
        System.out.println("SUBSCRIBER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw(){
        noLoop();
    }

    void subscriberCallback(final builtin_interfaces.msg.Time msg) {
        System.out.println("Received: " + msg);
        
        // once received a message, exit the program
        exit();
    }
}
