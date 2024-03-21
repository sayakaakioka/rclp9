/*
 * This Java source file was generated by the Gradle 'init' task.
 */
package rclp9;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.function.Consumer;

import org.junit.jupiter.api.Test;

import processing.core.*;

class RCLP9Test extends PApplet {
    private RCLP9 rcl;
    private int counter = 0;

    public RCLP9Test() {
        super();
        rcl = new RCLP9(this, "test_node_rclp9");
    }

    @Test
    public void testCreateAndDisposePublisher() {
        RCLP9 obj = new RCLP9(this, "test_create_and_dispose_publisher");
        obj.createPublisher(std_msgs.msg.String.class, "test_topic");
        assertEquals(obj.node().handle(), obj.publisher().nodeReference().get().handle());
        assertNotEquals(0, obj.publisher().nodeReference().get().handle());
        assertNotEquals(0, obj.publisher().handle());
        assertEquals(1, obj.node().publishers().size());

        obj.dispose();
        assertEquals(0, obj.publisher().handle());
        assertEquals(0, obj.node().publishers().size());
    }

    @Test
    public final void testCreateAndDisposeSubscriber() {
        RCLP9 obj = new RCLP9(this, "test_create_and_dispose_subscriber");
        obj.createSubscriber(std_msgs.msg.String.class, "test_topic", new Consumer<std_msgs.msg.String>() {
            public void accept(final std_msgs.msg.String msg) {
            }
        });
        assertEquals(obj.node().handle(), obj.subscriber().nodeReference().get().handle());
        assertNotEquals(0, obj.subscriber().nodeReference().get().handle());
        assertNotEquals(0, obj.subscriber().handle());
        assertEquals(1, obj.node().subscribers().size());

        obj.dispose();
        assertEquals(0, obj.subscriber().handle());
        assertEquals(0, obj.node().subscribers().size());
    }

    @Test
    public final void testFloat64Publisher() {
        rcl.createPublisher(std_msgs.msg.Float64.class, "test_topic_float64");
        rcl.createWallClockTimer(500, this::publisherCallback_Float64);
        rcl.spinOnce();
    }

    @Test
    public final void testPointPublisher() {
        rcl.createPublisher(geometry_msgs.msg.Point.class, "test_topic_point");
        rcl.createWallClockTimer(500, this::publisherCallback_Point);
        rcl.spinOnce();
    }

    @Test
    public final void testQuoternionPublisher() {
        rcl.createPublisher(geometry_msgs.msg.Quaternion.class, "test_topic_quoternion");
        rcl.createWallClockTimer(500, this::publisherCallback_Quaternion);
        rcl.spinOnce();
    }

    @Test
    public final void testPosePublisher() {
        rcl.createPublisher(geometry_msgs.msg.Pose.class, "test_topic_pose");
        rcl.createWallClockTimer(500, this::publisherCallback_Pose);
        rcl.spinOnce();
    }

    @Test
    public final void testTimePublisher() {
        rcl.createPublisher(builtin_interfaces.msg.Time.class, "test_topic_time");
        rcl.createWallClockTimer(500, this::publisherCallback_Time);
        rcl.spinOnce();
    }

    @Test
    public final void testHeaderPublisher() {
        rcl.createPublisher(std_msgs.msg.Header.class, "test_topic_header");
        rcl.createWallClockTimer(500, this::publisherCallback_Header);
        rcl.spinOnce();
    }

    @Test
    public final void testPoseStampedPublisher() {
        rcl.createPublisher(geometry_msgs.msg.PoseStamped.class, "test_topic_posestamped");
        rcl.createWallClockTimer(500, this::publisherCallback_PoseStamped);
        rcl.spinOnce();
    }

    @Test
    public final void testVariousDatatypeSubscribers() {
        RCLP9 obj_1 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_1.createSubscriber(std_msgs.msg.Float64.class, "test_topic_float64", new Consumer<std_msgs.msg.Float64>() {
            public void accept(final std_msgs.msg.Float64 msg) {
            }
        });
        obj_1.dispose();

        RCLP9 obj_2 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_2.createSubscriber(geometry_msgs.msg.Point.class, "test_topic_point",
                new Consumer<geometry_msgs.msg.Point>() {
                    public void accept(final geometry_msgs.msg.Point msg) {
                    }
                });
        obj_2.dispose();

        RCLP9 obj_3 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_3.createSubscriber(geometry_msgs.msg.Quaternion.class, "test_topic_quoternion",
                new Consumer<geometry_msgs.msg.Quaternion>() {
                    public void accept(final geometry_msgs.msg.Quaternion msg) {
                    }
                });
        obj_3.dispose();

        RCLP9 obj_4 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_4.createSubscriber(geometry_msgs.msg.Pose.class, "test_topic_pose", new Consumer<geometry_msgs.msg.Pose>() {
            public void accept(final geometry_msgs.msg.Pose msg) {
            }
        });
        obj_4.dispose();

        RCLP9 obj_5 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_5.createSubscriber(builtin_interfaces.msg.Time.class, "test_topic_time",
                new Consumer<builtin_interfaces.msg.Time>() {
                    public void accept(final builtin_interfaces.msg.Time msg) {
                    }
                });
        obj_5.dispose();

        RCLP9 obj_6 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_6.createSubscriber(std_msgs.msg.Header.class, "test_topic_time", new Consumer<std_msgs.msg.Header>() {
            public void accept(final std_msgs.msg.Header msg) {
            }
        });
        obj_6.dispose();

        RCLP9 obj_7 = new RCLP9(this, "test_varopis_datatype_subscribers");
        obj_7.createSubscriber(geometry_msgs.msg.PoseStamped.class, "test_topic_time",
                new Consumer<geometry_msgs.msg.PoseStamped>() {
                    public void accept(final geometry_msgs.msg.PoseStamped msg) {
                    }
                });
        obj_7.dispose();
    }

    @Test
    public final void testFunctionalPublisher() {
        rcl.createPublisher(std_msgs.msg.String.class, "test_topic");
        rcl.createWallClockTimer(500, this::publisherCallback);
        rcl.spinOnce();
    }

    @Test
    public final void testFunctionalSubscriber() {
        rcl.createSubscriber(std_msgs.msg.String.class, "test_topic", this::subscriberCallback);
        // This line will indefinitely pause the test until a message is received from
        // an external source.
        // rclp9.spinOnce();
    }

    private void publisherCallback() {
        var message = new std_msgs.msg.String();
        message.data = "Hello, world! " + this.counter;
        this.counter++;
        System.err.println("Publishing: [" + message.data + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Float64() {
        std_msgs.msg.Float64 message = new std_msgs.msg.Float64();
        message.data = (double) this.counter / 100000000;
        this.counter++;
        System.out.println("Publishing (Float64): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Point() {
        geometry_msgs.msg.Point message = new geometry_msgs.msg.Point();
        double val = (double) this.counter / 100000000;
        message.x = val;
        message.y = 2.0 * val;
        message.z = 4.0 * val;
        this.counter++;
        System.out.println("Publishing (Point): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Quaternion() {
        geometry_msgs.msg.Quaternion message = new geometry_msgs.msg.Quaternion();
        double val = (double) this.counter / 100000000;
        message.x = val;
        message.y = 2.0 * val;
        message.z = 4.0 * val;
        message.w = 8.0 * val;
        this.counter++;
        System.out.println("Publishing (Quaternion): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Pose() {
        geometry_msgs.msg.Pose message = new geometry_msgs.msg.Pose();
        double val = (double) this.counter / 100000000;
        message.position.x = val;
        message.position.y = 2.0 * val;
        message.position.z = 4.0 * val;
        message.orientation.x = 8.0 * val;
        message.orientation.y = 16.0 * val;
        message.orientation.z = 32.0 * val;
        message.orientation.w = 64.0 * val;
        this.counter++;
        System.out.println("Publishing (Pose): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Time() {
        builtin_interfaces.msg.Time message = new builtin_interfaces.msg.Time();
        message = rclp9.rcljava.time.Clock.now();
        System.out.println("Publishing (Time): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_Header() {
        std_msgs.msg.Header message = new std_msgs.msg.Header();
        message.stamp = rclp9.rcljava.time.Clock.now();
        message.frame_id = Integer.toString(this.counter);
        this.counter++;
        System.out.println("Publishing (Header): [" + message + "]");
        rcl.publish(message);
    }

    private void publisherCallback_PoseStamped() {
        geometry_msgs.msg.PoseStamped message = new geometry_msgs.msg.PoseStamped();

        message.header.stamp = rclp9.rcljava.time.Clock.now();
        message.header.frame_id = Integer.toString(this.counter);

        double val = (double) this.counter / 100000000;
        message.pose.position.x = val;
        message.pose.position.y = 2.0 * val;
        message.pose.position.z = 4.0 * val;
        message.pose.orientation.x = 8.0 * val;
        message.pose.orientation.y = 16.0 * val;
        message.pose.orientation.z = 32.0 * val;
        message.pose.orientation.w = 64.0 * val;

        this.counter++;
        System.out.println("Publishing (PoseStamped): [" + message + "]");
        rcl.publish(message);
    }

    private void subscriberCallback(final std_msgs.msg.String msg) {
        System.out.println("Received: " + msg);
    }
}
