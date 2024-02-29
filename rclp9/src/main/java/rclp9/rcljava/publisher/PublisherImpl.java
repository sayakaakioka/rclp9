package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Supplier;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.consumers.Consumer;
import rclp9.rcljava.events.EventHandler;
import rclp9.rcljava.events.EventHandlerImpl;
import rclp9.rcljava.events.EventStatus;
import rclp9.rcljava.events.PublisherEventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.utils.JNIUtils;

public class PublisherImpl<T extends MessageDefinition> implements Publisher<T> {
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

    private WeakReference<Node> nodeReference;
    private long handle;
    private String topic;
    private Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers;

    public PublisherImpl(final WeakReference<Node> nodeReference, final long handle, final String topic) {
        this.nodeReference = nodeReference;
        this.handle = handle;
        this.topic = topic;
        this.eventHandlers = new LinkedBlockingQueue<EventHandler<? extends EventStatus, ? extends Disposable>>();
    }

    @Override
    public void dispose() {
        for (var eventHandler : this.eventHandlers) {
            eventHandler.dispose();
        }
        this.eventHandlers.clear();

        Optional<Node> node = Optional.ofNullable(this.nodeReference.get());
        node.ifPresent((n) -> {
            n.removePublisher(this);
            nativeDispose(n.getHandle(), this.handle);
            this.handle = 0;
        });
    }

    @Override
    public final long getHandle() {
        return this.handle;
    }

    @Override
    public final WeakReference<Node> getNodeReference() {
        return this.nodeReference;
    }

    @Override
    public final void publish(final MessageDefinition message) {
        nativePublish(this.handle, message.getDestructorInstance(), message);
    }

    @Override
    public <S extends PublisherEventStatus> EventHandler<S, Publisher<T>> createEventHandler(Supplier<S> factory,
            Consumer<S> callback) {
        final var weakEventHandlers = new WeakReference<Collection<EventHandler<? extends EventStatus, ? extends Disposable>>>(
                this.eventHandlers);
        var disposeCallback = new Consumer<EventHandler<S, Publisher<T>>>() {
            public void accept(EventHandler<S, Publisher<T>> eventHandler) {
                var eventHandlers = weakEventHandlers.get();
                eventHandlers.remove(eventHandler);

            }
        };

        var status = factory.get();
        var eventHandle = nativeCreateEvent(this.handle, status.getPublisherEventType());
        var eventHandler = new EventHandlerImpl<S, Publisher<T>>(new WeakReference<Publisher<T>>(this), eventHandle,
                factory, callback, disposeCallback);
        this.eventHandlers.add(eventHandler);
        return eventHandler;
    }

    @Override
    public final void removeEventHandler(
            EventHandler<? extends PublisherEventStatus, Publisher<T>> eventHandler) {
        if (this.eventHandlers.remove(eventHandler)) {
            throw new IllegalArgumentException("The passed eventHandler was not created by this publisher");
        }
        eventHandler.dispose();
    }

    @Override
    public final Collection<EventHandler<? extends EventStatus, ? extends Disposable>> getEventHandlers() {
        return this.eventHandlers;
    }

    private static native long nativeCreateEvent(long handle, int eventType);

    private static native void nativeDispose(long nodeHandle, long handle);

    private static native <T extends MessageDefinition> void nativePublish(long handle, long messageDestructor,
            T message);

}
