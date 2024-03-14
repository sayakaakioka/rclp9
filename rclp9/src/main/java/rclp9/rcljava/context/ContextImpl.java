package rclp9.rcljava.context;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides an implementation of the Context interface.
 */
public final class ContextImpl implements Context {
    private static final Logger logger = Logger.getLogger(new Object(){}.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object(){}.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private long handle;

    /**
     * Instantiates a new context object using the specified handle.
     * @param handle ROS2 handle of this context
     */
    public ContextImpl(final long handle) {
        this.handle = Objects.requireNonNull(handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        shutdown();
        nativeDispose(this.handle);
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
     * {@inheritDoc}
     */
    @Override
    public final void init() {
        nativeInit(this.handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final boolean isValid() {
        return nativeIsValid(this.handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void shutdown() {
        nativeShutdown(this.handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final String toString() {
        return ("Instance of " + (new Object(){}.getClass().getName()) + ", handle = " + handle);
    }

    private static native void nativeDispose(long handle);

    private static native void nativeInit(long handle);

    private static native void nativeShutdown(long handle);

    private static native boolean nativeIsValid(long handle);
}
