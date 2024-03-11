package rclp9.rcljava.interfaces;

public interface Disposable {
    /**
     * Destroy the underlying ROS2 structure
     */
    void dispose();

    /**
     * Get a handle in ROS2 structure.
     *
     * @return A pointer to the underlying ROS2 structure
     */
    long handle();

    /**
     * Return the information of this object.
     *
     * @return Object information in String.
     */
    String toString();
}
