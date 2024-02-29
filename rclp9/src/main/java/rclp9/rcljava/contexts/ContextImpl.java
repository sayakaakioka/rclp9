package rclp9.rcljava.contexts;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.interfaces.Context;
import rclp9.rcljava.utils.JNIUtils;

public class ContextImpl implements Context {
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

    private long handle; // a pointer to the underlying context structure (rcl_context_t)

    public ContextImpl(final long handle){
        this.handle = handle;
    }

    @Override
    public void dispose() {
        shutdown();
        nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public long getHandle() {
        return this.handle;
    }

    @Override
    public void init() {
        nativeInit(this.handle);
    }

    @Override
    public void shutdown() {
        nativeShutdown(this.handle);
    }

    @Override
    public boolean isValid() {
        return nativeIsValid(this.handle);
    }

    private static native void nativeDispose(long handle);
    private static native void nativeInit(long handle);
    private static native void nativeShutdown(long handle);
    private static native boolean nativeIsValid(long handle);
}
