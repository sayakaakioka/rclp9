package rclp9.rcljava.node;

import java.lang.ref.WeakReference;

import java.util.Collection;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.interfaces.Context;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.publisher.Publisher;
import rclp9.rcljava.publisher.PublisherImpl;
import rclp9.rcljava.time.Clock;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.time.WallClockTimer;
import rclp9.rcljava.time.WallClockTimerImpl;
import rclp9.rcljava.utils.JNIUtils;

public class NodeImpl implements Node {
    private static final Logger logger = Logger.getLogger(NodeImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(NodeImpl.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private long handle;
    private Context context;
    private Collection<Publisher<? extends MessageDefinition>> publishers;
    private Clock clock;
    private Collection<Timer> timers;
    private Object parametersMutex;
    private boolean allowUndeclaredParameters;
    private Object parameterCallbacksMutex;

    public NodeImpl(final long handle, final Context context, final boolean allowUndeclaredParameters) {
        this.handle = handle;
        this.context = context;
        this.allowUndeclaredParameters = allowUndeclaredParameters;
        this.publishers = new LinkedBlockingQueue<Publisher<? extends MessageDefinition>>();
        this.clock = new Clock();
        this.timers = new LinkedBlockingQueue<Timer>();
        this.parametersMutex = new Object();
        this.parameterCallbacksMutex = new Object();
    }

    @Override
    public void dispose() {
        cleanup();
        nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public long getHandle() {
        return this.handle;
    }

    @Override
    public Collection<Publisher<? extends MessageDefinition>> getPublishers() {
        return this.publishers;
    }

    @Override
    public Collection<Timer> getTimers() {
        return this.timers;
    }

    @Override
    public <T extends MessageDefinition> Publisher<T> createPublisher(final Class<T> messageType, final String topic) {
        var qosProfileHandle = RCLJava.convertQoSProfileToHandle();
        var publisherHandle = nativeCreatePublisherHandle(this.handle, messageType, topic, qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);

        var publisher = new PublisherImpl<T>(new WeakReference<Node>(this), publisherHandle, topic);
        this.publishers.add(publisher);
        return publisher;
    }

    public WallClockTimer createWallClockTimer(final long period, final TimeUnit unit, final Callback callback) {
        var timerPeriodNS = TimeUnit.NANOSECONDS.convert(period, unit);
        var timerHandle = nativeCreateTimerHandle(clock.getHandle(), context.getHandle(), timerPeriodNS);
        var timer = new WallClockTimerImpl(new WeakReference<Node>(this), timerHandle, callback, timerPeriodNS);
        this.timers.add(timer);
        return timer;
    }

    @Override
    public boolean removePublisher(final Publisher<? extends MessageDefinition> publisher) {
        return this.publishers.remove(publisher);
    }

    private void cleanup() {
        cleanupDisposables(publishers);
        cleanupDisposables(timers);
    }

    private <T extends Disposable> void cleanupDisposables(Collection<T> disposables) {
        for (Disposable disposable : disposables) {
            disposable.dispose();
        }
        disposables.clear();
    }

    private static native long nativeCreateTimerHandle(long clockHandle, long contextHandle, long timerPeriod);

    private static native <T extends MessageDefinition> long nativeCreatePublisherHandle(long handle,
            Class<T> messageType, String topic, long qosProfileHandle);

    private static native void nativeDispose(long handle);

}
