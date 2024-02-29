package rclp9.rcljava.executors;

import rclp9.rcljava.events.EventHandler;
import rclp9.rcljava.events.EventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.time.Timer;

public class AnyExecutable {
    public Timer timer;
    public EventHandler<? extends EventStatus, ? extends Disposable> eventHandler;
}
