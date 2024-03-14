package rclp9.rcljava.time;

import rclp9.rcljava.interfaces.Disposable;

/**
 * This interface defines the APIs for timer related utilities.
 */
public interface Timer extends Disposable {
    /**
     * Calls RCL timer.
     */
    void callTimer();

    /**
     * Executes the callback function.
     */
    void executeCallback();

    /**
     * Checks whether the timer is ready or not.
     * @return true if the timer is ready, otherwise
     */
    boolean isReady();
}
