package rclp9.rcljava.time;

import rclp9.rcljava.interfaces.Disposable;

public interface Timer extends Disposable {
    void callTimer();

    void executeCallback();

    boolean isReady();
}
