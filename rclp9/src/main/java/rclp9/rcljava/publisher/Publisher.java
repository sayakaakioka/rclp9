package rclp9.rcljava.publisher;

import java.lang.ref.WeakReference;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.Node;

/**
 * This interface defines the APIs of publishers.
 */
public interface Publisher<T1 extends MessageDefinition> extends Disposable {
    /**
     * Returns a reference to the node which this publisher belongs to.
     *
     * @return a weak reference to the node
     */
    WeakReference<Node> nodeReference();

    /**
     * Publishes a message.
     *
     * @param message a message to be published
     */
    void publish(T1 message);
}
