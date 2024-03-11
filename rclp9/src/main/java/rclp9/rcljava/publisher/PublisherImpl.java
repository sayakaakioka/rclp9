package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.event.EventHandler;
import rclp9.rcljava.event.EventHandlerImpl;
import rclp9.rcljava.event.EventStatus;
import rclp9.rcljava.event.PublisherEventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.util.JNIUtils;

public final class PublisherImpl<T1 extends MessageDefinition> implements Publisher<T1> {
    private static final Logger logger = Logger.getLogger(PublisherImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(PublisherImpl.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private final WeakReference<Node> nodeReference;
    private long handle;
    private final String topic;
    private final Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers;

    public PublisherImpl(final WeakReference<Node> nodeReference, final long handle, final String topic) {
        this.nodeReference = Objects.requireNonNull(nodeReference);
        this.handle = Objects.requireNonNull(handle);
        this.topic = Objects.requireNonNull(topic);
        this.eventHandlers = new LinkedBlockingQueue<>();
    }

    @Override
    public final <T2 extends PublisherEventStatus> EventHandler<T2, Publisher<T1>> createEventHandler(
            Supplier<T2> factory, Consumer<T2> callback) {
        var weakEventHandlers = new WeakReference<Collection<EventHandler<? extends EventStatus, ? extends Disposable>>>(
                this.eventHandlers);
        var disposeCallback = new Consumer<EventHandler<T2, Publisher<T1>>>() {
            public void accept(EventHandler<T2, Publisher<T1>> eventHandler) {
                var eventHandlers = weakEventHandlers.get();
                eventHandlers.remove(eventHandler);

            }
        };

        var status = factory.get();
        var eventHandle = nativeCreateEvent(this.handle, status.publisherEventType());
        var eventHandler = new EventHandlerImpl<T2, Publisher<T1>>(new WeakReference<Publisher<T1>>(this), eventHandle,
                factory, callback, disposeCallback);
        this.eventHandlers.add(eventHandler);
        return eventHandler;
    }

    @Override
    public final void dispose() {
        for (var eventHandler : this.eventHandlers) {
            eventHandler.dispose();
        }
        this.eventHandlers.clear();

        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresent((n) -> {
            n.removePublisher(this);
            nativeDispose(n.handle(), this.handle);
            this.handle = 0;
        });
    }

    @Override
    public final Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers() {
        return this.eventHandlers;
    }

    @Override
    public final long handle() {
        return this.handle;
    }

    @Override
    public final WeakReference<Node> nodeReference() {
        return this.nodeReference;
    }

    @Override
    public final void publish(final MessageDefinition message) {
        nativePublish(this.handle, message.getDestructorInstance(), message);
    }

    @Override
    public final <T2 extends PublisherEventStatus> void removeEventHandler(
            EventHandler<T2, Publisher<T1>> eventHandler) {
        if (this.eventHandlers.remove(eventHandler)) {
            throw new IllegalArgumentException("The passed eventHandler was not created by this publisher");
        }
        eventHandler.dispose();
    }

    @Override
    public final String toString() {
        return ("Instance of " + PublisherImpl.class.getName() + ", handle = " + handle + ", topic = " + topic);
    }

    private static native long nativeCreateEvent(long handle, int eventType);

    private static native void nativeDispose(long nodeHandle, long handle);

    private static native <T extends MessageDefinition> void nativePublish(long handle, long messageDestructor,
            T message);

}
