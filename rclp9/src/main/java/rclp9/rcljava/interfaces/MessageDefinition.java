package rclp9.rcljava.interfaces;

/**
 * This interface defines the APIs for messages exchanged between 
 * publishers, and subscribers.
 * The methods defined in this interface are primarily used for type translations 
 * between Java and C++. These methods are often invoked from native libraries.
 * Type specific methods should be defined in each interface that inherits this interface.
 */
public interface MessageDefinition {
    /**
     * Returns a ROS message object.
     * @return ID of the message instance
     */
    public long getFromJavaConverterInstance();

    /**
     * Returns a Java message object.
     * @return ID of the message instance
     */
    public long getToJavaConverterInstance();

    /**
     * Returns a ROSIDL type support instance.
     * @return ID of the type support instance.
     */
    public long getTypeSupportInstance();

    /**
     * Gets the destructor instance of this message definition.
     * @return ID of the destructor
     */
    public long getDestructorInstance();
}
