package rclp9.rcljava.time;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides an implementation of the Timer interface.
 */
public final class TimerImpl implements Timer {
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

    private final WeakReference<Node> nodeReference;
    private long handle;
    private final Callback callback;
    private final long timerPeriodNS;

    /**
     * Instantiates a new timer object.
     * @param nodeReference　a weak reference to the associated node
     * @param handle the ROS2 handle associated with this timer
     * @param callback a callback function
     * @param timerPeriodNS the time interval specified in nanoseconds
     */
    public TimerImpl(final WeakReference<Node> nodeReference, final long handle, final Callback callback,
            final long timerPeriodNS) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.callback = Objects.requireNonNull(callback);
        this.timerPeriodNS = Objects.requireNonNull(timerPeriodNS);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void callTimer() {
        nativeCallTimer(this.handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void executeCallback() {
        this.callback.call();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final boolean isReady() {
        return nativeIsReady(this.handle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresentOrElse(
                (n) -> {
                    nativeDispose(this.handle);
                    this.handle = 0;
                },
                () -> {
                    logger.info("Node object is null.");
                });
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
    public final String toString() {
        return ("Instance of " + (new Object(){}.getClass().getName()) + ", handle = " + handle);
    }

    private static native void nativeCallTimer(long handle);

    private static native void nativeDispose(long handle);

    private static native boolean nativeIsReady(long handle);

}
