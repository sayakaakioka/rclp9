package rclp9.rcljava.interfaces;

/**
 * This interface defines the APIs for the underlying ROS2 structure.
 */
public interface Disposable {
    /**
     * Destroys the underlying structure.
     */
    void dispose();

    /**
     * Gets a handle in ROS2 structure.
     * @return ID in ROS2 structure
     */
    long handle();

    /**
     * Returns a string containing the information of this object. 
     * The details should be determined in the interfaces inherited this interface.
     * @return string representation of this object
     */
    String toString();
}
