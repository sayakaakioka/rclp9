package rclp9.rcljava.node;

/**
 * This interface defines the APIs of functional nodes.
 */
public interface ComposableNode {
    /**
     * Returns the node instance maintained by this class.
     * @return the node instance
     */
    Node node();

    /**
     * Returns a string containing information about this node.
     * The expected format is as follows: "Instance of (class name), name = (name), handle = (handle in ROS2)".
     * @return a string containing the information
     */
    String toString();
}
