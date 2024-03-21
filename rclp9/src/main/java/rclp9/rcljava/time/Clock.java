package rclp9.rcljava.time;

import java.util.concurrent.TimeUnit;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class creates and manages ROS2 handle for a clock.
 */
public final class Clock implements Disposable {
    private static final Logger logger = Logger.getLogger(new Object() {
    }.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object() {
            }.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private long handle;

    /**
     * Creates an instance of this class, and obtains a clock handle.
     */
    public Clock() {
        this.handle = Objects.requireNonNull(Clock.nativeCreateClockHandle());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        Clock.nativeDispose(this.handle);
        this.handle = 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long handle() {
        return this.handle;
    }

    /**
     * Gets current time in builtin_interfaces.msg.Time.
     *
     * @return current time
     */
    public final static builtin_interfaces.msg.Time now() {
        long rclTime = nativeRCLSystemTimeNow();
        builtin_interfaces.msg.Time msgTime = new builtin_interfaces.msg.Time();
        msgTime.sec = (int) (TimeUnit.SECONDS.convert(rclTime, TimeUnit.NANOSECONDS));
        msgTime.nanosec = (int) (rclTime % (1000 * 1000 * 1000));
        return msgTime;
    }

    private static native long nativeCreateClockHandle();

    private static native void nativeDispose(long handle);

    private static native long nativeRCLSystemTimeNow();

}
