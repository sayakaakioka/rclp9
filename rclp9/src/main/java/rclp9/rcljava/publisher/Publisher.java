package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.function.Consumer;
import java.util.function.Supplier;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.event.EventHandler;
import rclp9.rcljava.event.EventStatus;
import rclp9.rcljava.event.PublisherEventStatus;

public interface Publisher<T1 extends MessageDefinition> extends Disposable {
    /**
     * Create an event handler.
     *
     * @param <T>      A publisher event status type.
     * @param factory  A factory that instantiates an event status of type T.
     * @param callback Callback that will be called when triggered.
     * @return
     */
    <T2 extends PublisherEventStatus> EventHandler<T2, Publisher<T1>> createEventHandler(Supplier<T2> factory,
            Consumer<T2> callback);

    /**
     * Returns all the event handlers in this Publisher.
     *
     * @return The registered event handlers.
     */
    Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers();

    /**
     * A reference to the node that is created by this publisher.
     *
     * @return a @{link java.lang.ref.WeakReference} to the node
     */
    WeakReference<Node> nodeReference();

    /**
     * Publish a message
     *
     * @param message a message to be published
     */
    void publish(final MessageDefinition message);

    /**
     * Remove an event handler.
     *
     * @param <T>          A publisher's event status type.
     * @param eventHandler The event handler to be removed.
     */
    <T2 extends PublisherEventStatus> void removeEventHandler(EventHandler<T2, Publisher<T1>> eventHandler);
}
