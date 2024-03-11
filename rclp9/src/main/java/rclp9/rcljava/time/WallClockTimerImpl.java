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

public final class WallClockTimerImpl implements WallClockTimer {
    private static final Logger logger = Logger.getLogger(WallClockTimerImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(WallClockTimerImpl.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private final WeakReference<Node> nodeReference;
    private long handle;
    private final Callback callback;
    private final long timerPeriodNS;

    public WallClockTimerImpl(final WeakReference<Node> nodeReference, final long handle, final Callback callback,
            final long timerPeriodNS) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.callback = Objects.requireNonNull(callback);
        this.timerPeriodNS = Objects.requireNonNull(timerPeriodNS);
    }

    @Override
    public final void callTimer() {
        nativeCallTimer(this.handle);
    }

    @Override
    public final void executeCallback() {
        this.callback.call();
    }

    @Override
    public final boolean isReady() {
        return nativeIsReady(this.handle);
    }

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

    @Override
    public final long handle() {
        return this.handle;
    }

    @Override
    public final boolean isCancelled() {
        throw new UnsupportedOperationException("Unimplemented method 'isCancelled()'");
    }

    @Override
    public final void cancel() {
        throw new UnsupportedOperationException("Unimplemented method 'cancel()'");
    }

    @Override
    public final void reset() {
        throw new UnsupportedOperationException("Unimplemented method 'reset()'");
    }

    @Override
    public final long timerPeriodNS() {
        throw new UnsupportedOperationException("Unimplemented method 'getTimerPeriodNS()'");
    }

    @Override
    public final void timerPeriodNS(long period) {
        throw new UnsupportedOperationException("Unimplemented method 'setTimerPeriodNS()'");
    }

    @Override
    public final long timeSinceLastCall() {
        throw new UnsupportedOperationException("Unimplemented method 'timeSinceLastCall()'");
    }

    @Override
    public final long timeUntilNextCall() {
        throw new UnsupportedOperationException("Unimplemented method 'timeUntilNextCall()'");
    }

    @Override
    public final String toString() {
        return ("Instance of " + WallClockTimerImpl.class.getName() + ", handle = " + handle);
    }

    private static native void nativeCallTimer(long handle);

    private static native void nativeDispose(long handle);

    private static native boolean nativeIsReady(long handle);

}
