package rclp9.rcljava;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.BeforeAll;
//import org.junit.jupiter.api.Test;

public class RCLJavaTest {
    @BeforeAll
    public final static void testCreateAndDispose() {
        RCLJava.init();
        var node = RCLJava.createNode("test_node_rcljava");
        var publisher = node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class,
                "test_topic");
        assertEquals(node.getHandle(), publisher.getNodeReference().get().getHandle());
        assertNotEquals(0, publisher.getNodeReference().get().getHandle());
        assertNotEquals(0, publisher.getHandle());
        assertEquals(1, node.getPublishers().size());

        // We expect that calling dispose should result in a zero handle
        // and the reference is dropped from the Node
        publisher.dispose();
        assertEquals(0, publisher.getHandle());
        assertEquals(0, node.getPublishers().size());

        RCLJava.shutdown();
    }
}
