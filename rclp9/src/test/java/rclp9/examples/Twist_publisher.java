package rclp9.examples;

import processing.core.PApplet;
import rclp9.*;

public class Twist_publisher {
    public static void main(String[] args) {
        PApplet.runSketch(new String[] { "TwistPublisher" }, new TwistPublisher());
    }
}

class TwistPublisher extends PApplet {
    RCLP9 rclp9 = new RCLP9(this, "rclp9_Twist_test_publisher");
   
    @Override
    public void settings() {
        size(10, 10, JAVA2D);
    }

    @Override
    public void setup() {
        rclp9.createPublisher(geometry_msgs.msg.Twist.class, "twist_test_topic");
        rclp9.createWallClockTimer(500, this::timerCallback);
        System.out.println("PUBLISHER READY");
        rclp9.spinOnce();
    }

    @Override
    public void draw() {
        noLoop();
    }

    void timerCallback() {
        geometry_msgs.msg.Twist message = new geometry_msgs.msg.Twist();
        double val = (double)1.0/100000000;
        message.linear.x = val;
        message.linear.y = 2.0*val;
        message.linear.z = 4.0*val;
        message.angular.x = 8.0*val;
        message.angular.y = 16.0*val;
        message.angular.z = 32.0*val;
        
        System.out.println("Publishing: [" + message + "]");
        rclp9.publish(message);

        // once published a message, exit the program
        System.out.println("PUBLISHER DONE");
        System.out.flush();
        exit();
    }
}
