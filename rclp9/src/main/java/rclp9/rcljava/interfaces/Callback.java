package rclp9.rcljava.interfaces;

/**
 * This interface defines API for callback functions.
 * The definition here should be utilized only if the callback function takes no parameters, 
 * and returns no value. 
 * Currently, only a timer accepts this type of instance.
 */
@FunctionalInterface
public interface Callback {
    /**
     * The functional interface.
     */
    void call();
}
