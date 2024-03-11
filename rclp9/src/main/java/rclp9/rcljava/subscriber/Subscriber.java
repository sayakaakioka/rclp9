package rclp9.rcljava.subscriber;

import java.lang.ref.WeakReference;
import java.util.Collection;

import rclp9.rcljava.event.EventHandler;
import rclp9.rcljava.event.EventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;

public interface Subscriber<T extends MessageDefinition> extends Disposable {

    void executeCallback(T message);

    /**
     * Get the event handlers that were registered in this Subscription.
     *
     * @return The registered event handlers.
     */
    Collection<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandlers();

    /**
     * The type of messages this subscriber receives.
     *
     * @return The message type.
     */
    Class<T> messageType();

    /**
     * A reference to the node that is created by this publisher.
     *
     * @return A @{link java.lang.ref.WeakReference} to the node
     */
    WeakReference<Node> nodeReference();

}
