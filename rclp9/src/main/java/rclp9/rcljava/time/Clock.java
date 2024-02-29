package rclp9.rcljava.time;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.utils.JNIUtils;

public class Clock implements Disposable {
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
        this.handle = Clock.nativeCreateClockHandle();
    }

    @Override
    public void dispose() {
        throw new UnsupportedOperationException("Unimplemented method 'dispose()'");
        /*
         * Clock.nativeDispose(this.handle);
         * this.handle = 0;
         */
    }

    @Override
    public long getHandle() {
        return this.handle;
    }

    private static native long nativeCreateClockHandle();

    private static native void nativeDispose(long handle);

}
