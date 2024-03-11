package rclp9.rcljava.time;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.util.JNIUtils;

public final class Clock implements Disposable {
    private static final Logger logger = Logger.getLogger(Clock.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(Clock.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private long handle;

    public Clock() {
        this.handle = Objects.requireNonNull(Clock.nativeCreateClockHandle());
    }

    @Override
    public final void dispose() {
        Clock.nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public final long handle() {
        return this.handle;
    }

    private static native long nativeCreateClockHandle();

    private static native void nativeDispose(long handle);

}
