package rclp9.rcljava.subscriber;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides an implementation of the Subscriber interface.
 */
public class SubscriberImpl<T extends MessageDefinition> implements Subscriber<T> {
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
    private final Class<T> messageType;
    private final String topic;
    private final Consumer<T> callback;

    /**
     * Instantiates a new subscriber object.
     * @param nodeReference a weak reference to the associated node
     * @param handle the ROS2 handle associated with this timer
     * @param messageType the type of the messages
     * @param topic a topic name
     * @param callback a callback function
     */
    public SubscriberImpl(final WeakReference<Node> nodeReference, final long handle, final Class<T> messageType,
            final String topic, final Consumer<T> callback) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.messageType = Objects.requireNonNull(messageType);
        this.topic = Objects.requireNonNull(topic);
        this.callback = Objects.requireNonNull(callback);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresent((n) -> {
            n.removeSubscriber(this);
            nativeDispose(n.handle(), this.handle);
            this.handle = 0;
        });
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void executeCallback(T message) {
        this.callback.accept(message);
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
    public final Class<T> messageType() {
        return messageType;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final WeakReference<Node> nodeReference() {
        return this.nodeReference;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final String toString() {
        return ("Instance of " + (new Object(){}.getClass().getName()) + ", handle = " + handle + ", topic = " + topic
                + ", messageType = " + messageType.getName());
    }

    private static native void nativeDispose(long nodeHandle, long handle);

}
