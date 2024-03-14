package rclp9.rcljava.context;

import rclp9.rcljava.interfaces.Disposable;

/**
 * This interface defines the APIs for ROS2 contexts.
 */
public interface Context extends Disposable {
    /**
     * Initializes the ROS2 context. 
     * This method should be called before any creation of 
     * nodes, publishers, subscribers, 
     * and any other related objects.
     */
    void init();

    /**
     * Shuts down the context.
     * This method should be called before everything ends 
     * to ensure no leftovers are left behind.
     */
    void shutdown();

    /**
     * Checks whether the context is valid or not.
     * @return true if the context is valid, and false otherwise
     */
    boolean isValid();

    /**
     * Returns a string containing information about this context.
     * The expected format is as follows: "Instance of (class name), handle = (handle in ROS2)".
     * @return a string containing the information
     */
    String toString();
}
