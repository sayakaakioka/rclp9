package rclp9.rcljava.subscriber;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Consumer;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.event.EventHandler;
import rclp9.rcljava.event.EventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.util.JNIUtils;

public class SubscriberImpl<T extends MessageDefinition> implements Subscriber<T> {
    private static final Logger logger = Logger.getLogger(SubscriberImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(SubscriberImpl.class);
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

    private final Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers;

    public SubscriberImpl(final WeakReference<Node> nodeReference, final long handle, final Class<T> messageType,
            final String topic, final Consumer<T> callback) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.messageType = Objects.requireNonNull(messageType);
        this.topic = Objects.requireNonNull(topic);
        this.callback = Objects.requireNonNull(callback);
        this.eventHandlers = new LinkedBlockingQueue<>();
    }

    @Override
    public final void dispose() {
        for (var eventHandler : this.eventHandlers) {
            eventHandler.dispose();
        }

        this.eventHandlers.clear();
        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresent((n) -> {
            n.removeSubscriber(this);
            nativeDispose(n.handle(), this.handle);
            this.handle = 0;
        });
    }

    @Override
    public final Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers() {
        return this.eventHandlers;
    }

    @Override
    public final void executeCallback(T message) {
        this.callback.accept(message);
    }

    @Override
    public final long handle() {
        return this.handle;
    }

    @Override
    public final Class<T> messageType() {
        return messageType;
    }

    @Override
    public final WeakReference<Node> nodeReference() {
        return this.nodeReference;
    }

    @Override
    public final String toString() {
        return ("Instance of " + SubscriberImpl.class.getName() + ", handle = " + handle + ", topic = " + topic
                + ", messageType = " + messageType.getName());
    }

    private static native void nativeDispose(long nodeHandle, long handle);

}
