package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Float64_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "Float64Publisher" }, new Float64Publisher());
    }
}

class Float64Publisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "rclp9_Float64_test_publisher");
    
    @Override
    public void settings() {
        size(10, 10);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        rclp9.createPublisher(std_msgs.msg.Float64.class, "float64_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }
    
    @Override
    public void draw() {
        noLoop();
    }

    void timerCallback() {
        std_msgs.msg.Float64 message = new std_msgs.msg.Float64();
        message.data = Math.PI;
        System.out.println("Publishing: [" + message.data + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }
}
