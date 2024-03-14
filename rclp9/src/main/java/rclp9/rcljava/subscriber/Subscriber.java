package rclp9.rcljava.subscriber;

import java.lang.ref.WeakReference;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;

/**
 * This interface defines the APIs of subscribers.
 */
public interface Subscriber<T extends MessageDefinition> extends Disposable {

    /**
     * Executes the callback function on arrival of the message.
     * @param message message to be processed
     */
    void executeCallback(T message);

    /**
     * Returns the type of messages this subscriber receives.
     *
     * @return the class of the message
     */
    Class<T> messageType();

    /**
     * Returns a reference to the node which this subscriber belongs to.
     *
     * @return A weak reference to the node
     */
    WeakReference<Node> nodeReference();

}
