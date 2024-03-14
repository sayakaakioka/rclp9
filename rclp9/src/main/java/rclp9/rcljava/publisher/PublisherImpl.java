package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides an implementation of the Publisher interface.
 */
public final class PublisherImpl<T1 extends MessageDefinition> implements Publisher<T1> {
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
    private final String topic;

    /**
     * Instantiates a new publisher object.
     * @param nodeReference a weak reference to the associated node
     * @param handle the ROS2 handle associated with this timer
     * @param topic the topic name
     */
    public PublisherImpl(final WeakReference<Node> nodeReference, final long handle, final String topic) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.topic = Objects.requireNonNull(topic);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void dispose() {
        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresent((n) -> {
            n.removePublisher(this);
            nativeDispose(n.handle(), this.handle);
            this.handle = 0;
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
    public final WeakReference<Node> nodeReference() {
        return this.nodeReference;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void publish(final T1 message) {
        nativePublish(this.handle, message.getDestructorInstance(), message);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final String toString() {
        return ("Instance of " + (new Object(){}.getClass().getName()) + ", handle = " + handle + ", topic = " + topic);
    }

    private static native void nativeDispose(long nodeHandle, long handle);

    private static native <T extends MessageDefinition> void nativePublish(long handle, long messageDestructor,
            T message);

}
