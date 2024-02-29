package rclp9.rcljava.time;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.utils.JNIUtils;

public class WallClockTimerImpl implements WallClockTimer {
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
    private long timerPeriodNS;

    public WallClockTimerImpl(final WeakReference<Node> nodeReference, final long handle, final Callback callback,
            final long timerPeriodNS) {
        this.nodeReference = nodeReference;
        this.handle = handle;
        this.callback = callback;
        this.timerPeriodNS = timerPeriodNS;
    }

    @Override
    public void callTimer() {
        nativeCallTimer(this.handle);
    }

    @Override
    public void executeCallback() {
        logger.info("executeCallback() called");
        this.callback.call();
    }

    @Override
    public boolean isReady() {
        return nativeIsReady(this.handle);
    }

    @Override
    public void dispose() {
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
    public long getHandle() {
        return this.handle;
    }

    @Override
    public long getTimerPeriodNS() {
        throw new UnsupportedOperationException("Unimplemented method 'getTimerPeriodNS()'");
    }

    @Override
    public void setTimerPeriodNS(long period) {
        throw new UnsupportedOperationException("Unimplemented method 'setTimerPeriodNS()'");
    }

    @Override
    public boolean isCancelled() {
        throw new UnsupportedOperationException("Unimplemented method 'isCancelled()'");
    }

    @Override
    public void cancel() {
        throw new UnsupportedOperationException("Unimplemented method 'cancel()'");
    }

    @Override
    public void reset() {
        throw new UnsupportedOperationException("Unimplemented method 'reset()'");
    }

    @Override
    public long timeSinceLastCall() {
        throw new UnsupportedOperationException("Unimplemented method 'timeSinceLastCall()'");
    }

    @Override
    public long timeUntilNextCall() {
        throw new UnsupportedOperationException("Unimplemented method 'timeUntilNextCall()'");
    }

    private static native void nativeCallTimer(long handle);

    private static native void nativeDispose(long handle);

    private static native boolean nativeIsReady(long handle);

}
