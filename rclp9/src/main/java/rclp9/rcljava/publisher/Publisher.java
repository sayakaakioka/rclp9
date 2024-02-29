package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.function.Supplier;

import rclp9.rcljava.consumers.Consumer;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.events.EventHandler;
import rclp9.rcljava.events.EventStatus;
import rclp9.rcljava.events.PublisherEventStatus;

public interface Publisher<T extends MessageDefinition> extends Disposable {
    /**
     * Publish a message
     *
     * @param message a message to be published
     */
    void publish(final MessageDefinition message);

    /**
     * A reference to the node that created this publisher.
     *
     * @return a @{link java.lang.ref.WeakReference} to the node
     */
    WeakReference<Node> getNodeReference();

    /**
     * Create an event handler.
     *
     * @param <T>      A publisher event status type.
     * @param factory  A factory that instantiates an event status of type T.
     * @param callback Callback that will be called when triggered.
     * @return
     */
    <S extends PublisherEventStatus> EventHandler<S, Publisher<T>> createEventHandler(Supplier<S> factory,
            Consumer<S> callback);

    /**
     * Remove an event handler.
     *
     * @param <T>          A publisher's event status type.
     * @param eventHandler The event handler to be removed.
     */
    void removeEventHandler(EventHandler<? extends PublisherEventStatus, Publisher<T>> eventHandler);

    /**
     * Returns all the event handlers in this Publisher.
     *
     * @return The registered event handlers.
     */
    Collection<EventHandler<? extends EventStatus, ? extends Disposable>> getEventHandlers();
}
