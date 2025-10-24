package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Float64_subscriber {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "Float64Subscriber" }, new Float64Subscriber());
    }
}

class Float64Subscriber extends PApplet {
    @Override
    public void settings() {
        size(10, 10);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        RCLP9 rclp9 = new RCLP9(this, "rclp9_Float64_test_subscriber");
        rclp9.createSubscriber(std_msgs.msg.Float64.class, "float64_test_topic", this::subscriberCallback);
        System.out.println("SUBSCRIBER READY");
        rclp9.spinOnce();
    }
    
    @Override
    public void draw(){
        noLoop();
    }

    void subscriberCallback(final std_msgs.msg.Float64 msg) {
        System.out.println("Received: " + msg);

        // once received a message, exit the program
        exit();
    }
}
