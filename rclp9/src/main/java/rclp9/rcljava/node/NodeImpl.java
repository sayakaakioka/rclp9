package rclp9.rcljava.node;

import java.lang.ref.WeakReference;

import java.util.Collection;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.context.Context;
import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.publisher.Publisher;
import rclp9.rcljava.publisher.PublisherImpl;
import rclp9.rcljava.subscriber.Subscriber;
import rclp9.rcljava.subscriber.SubscriberImpl;
import rclp9.rcljava.time.Clock;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.time.TimerImpl;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides an implementation of the Node interface.
 */
public final class NodeImpl implements Node {
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
    private final Context context;
    private final Clock clock;
    // TODO: Consider that the message type may be a mix of several types.
    private final Collection<Publisher<? extends MessageDefinition>> publishers;
    private final Collection<Subscriber<? extends MessageDefinition>> subscribers;
    private final Collection<Timer> timers;

    /**
     * Instantiates a new node object.
     * @param handle the ROS2 handle associated with this node
     * @param context the ROS2 context associated with this node
     */
    public NodeImpl(final long handle, final Context context) {
        this.handle = Objects.requireNonNull(handle);
        this.context = Objects.requireNonNull(context);
        this.clock = new Clock();
        this.publishers = new LinkedBlockingQueue<>();
        this.subscribers = new LinkedBlockingQueue<>();
        this.timers = new LinkedBlockingQueue<>();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final <T extends MessageDefinition> Publisher<T> createPublisher(final Class<T> messageType,
            final String topic) {
        var qosProfileHandle = RCLJava.convertQoSProfileToHandle();
        var publisherHandle = nativeCreatePublisherHandle(this.handle, messageType, topic, qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);

        var publisher = new PublisherImpl<T>(new WeakReference<Node>(this), publisherHandle, topic);
        this.publishers.add(publisher);
        return publisher;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final <T extends MessageDefinition> Subscriber<T> createSubscriber(final Class<T> messageType,
            final String topic, final Consumer<T> callback) {
        var qosProfileHandle = RCLJava.convertQoSProfileToHandle();
        var subscriberHandle = nativeCreateSubscriberHandle(this.handle, messageType, topic, qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);

        var subscriber = new SubscriberImpl<T>(new WeakReference<Node>(this), subscriberHandle, messageType, topic,
                callback);
        this.subscribers.add(subscriber);
        return subscriber;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final Timer createTimer(final long period, final TimeUnit unit, final Callback callback) {
        var timerPeriodNS = TimeUnit.NANOSECONDS.convert(period, unit);
        var timerHandle = nativeCreateTimerHandle(clock.handle(), context.handle(), timerPeriodNS);
        var timer = new TimerImpl(new WeakReference<Node>(this), timerHandle, callback, timerPeriodNS);
        this.timers.add(timer);
        return timer;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        cleanup();
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
    public final Collection<Publisher<? extends MessageDefinition>> publishers() {
        return this.publishers;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final Collection<Subscriber<? extends MessageDefinition>> subscribers() {
        return this.subscribers;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final Collection<Timer> timers() {
        return this.timers;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final <T extends MessageDefinition> boolean removePublisher(final Publisher<T> publisher) {
        return this.publishers.remove(publisher);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final <T extends MessageDefinition> boolean removeSubscriber(final Subscriber<T> subscriber) {
        return this.subscribers.remove(subscriber);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final String toString() {
        return ("Instance of " + (new Object(){}.getClass().getName()) + ", handle = " + handle);
    }

    private void cleanup() {
        cleanupDisposables(publishers);
        cleanupDisposables(subscribers);
        cleanupDisposables(timers);
    }

    private <T extends Disposable> void cleanupDisposables(final Collection<T> disposables) {
        for (Disposable disposable : disposables) {
            disposable.dispose();
        }
        disposables.clear();
    }

    private static native <T extends MessageDefinition> long nativeCreatePublisherHandle(long handle,
            Class<T> messageType, String topic,
            long qosProfileHandle);

    private static native <T extends MessageDefinition> long nativeCreateSubscriberHandle(long handle,
            Class<T> messageType, String topic,
            long qosProfileHandle);

    private static native long nativeCreateTimerHandle(long clockHandle, long contextHandle, long timerPeriod);

    private static native void nativeDispose(long handle);
}
