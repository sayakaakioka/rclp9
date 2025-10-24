package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Vector3_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "Vector3Publisher" }, new Vector3Publisher());
    }
}

class Vector3Publisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "rclp9_Vector3_publisher");

    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        surface.setVisible(false);

        rclp9.createPublisher(geometry_msgs.msg.Vector3.class, "vector3_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }
    
    @Override
    public void draw() {
        noLoop();
    }

    void timerCallback() {
        geometry_msgs.msg.Vector3 message = new geometry_msgs.msg.Vector3();
        double val = (double)1.0/100000000;
        message.x = val;
        message.y = 2.0*val;
        message.z = 4.0*val;
        System.out.println("Publishing: [" + message + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }
}
