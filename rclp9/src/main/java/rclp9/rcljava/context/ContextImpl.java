package rclp9.rcljava.context;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.Context;
import rclp9.rcljava.util.JNIUtils;

public final class ContextImpl implements Context {
    private static final Logger logger = Logger.getLogger(ContextImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(ContextImpl.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private long handle;

    public ContextImpl(final long handle) {
        this.handle = Objects.requireNonNull(handle);
    }

    @Override
    public final void dispose() {
        shutdown();
        nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public final long handle() {
        return this.handle;
    }

    @Override
    public final void init() {
        nativeInit(this.handle);
    }

    @Override
    public final boolean isValid() {
        return nativeIsValid(this.handle);
    }

    @Override
    public final void shutdown() {
        nativeShutdown(this.handle);
    }

    @Override
    public final String toString() {
        return ("Instance of " + ContextImpl.class.getName() + ", handle = " + handle);
    }

    private static native void nativeDispose(long handle);

    private static native void nativeInit(long handle);

    private static native void nativeShutdown(long handle);

    private static native boolean nativeIsValid(long handle);
}
