package rclp9.rcljava.interfaces;

public interface Disposable {
    /**
     * destroy the underlying ROS2 structure
     */
    void dispose();

    /**
     * @return a pointer to the underlying ROS2 structure
     */
    long getHandle();
}
