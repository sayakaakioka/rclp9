package rclp9.rcljava.node;

import java.util.Collection;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.publisher.Publisher;
import rclp9.rcljava.subscriber.Subscriber;
import rclp9.rcljava.time.Timer;

/**
 * This class is comparable with rcl_node_t in ROS2.
 * A node should be created via @{link RCLJava#createNode(String).}
 */
public interface Node extends Disposable {
    /**
     * Creates a Publisher instance.
     *
     * @param <T>         the type of the messages to be published
     * @param messageType the class of the messages to be published
     * @param topic       the topic to which the messages will be published
     * @return an instance of the publisher
     */
    <T extends MessageDefinition> Publisher<T> createPublisher(final Class<T> messageType, final String topic);

    /**
     * Creates a Subscriber instance.
     *
     * @param <T>         the type of the messages to be subscribed
     * @param messageType the class of the messages to be subscribed
     * @param topic       the topic where the messages will be subscribed
     * @param callback    the callback function to be called upon arrival of a message
     * @return an instance of the subscriber
     */
    <T extends MessageDefinition> Subscriber<T> createSubscriber(final Class<T> messageType, final String topic,
            final Consumer<T> callback);

    /**
     * Creates a Timer instance.
     *
     * @param period   the time interval
     * @param unit     the unit for the period
     * @param callback the callback function that will be called
     * @return an instance of the timer
     */
    Timer createTimer(final long period, final TimeUnit unit, final Callback callback);

    /**
     * Returns a list of all publishers created by this instance.
     * @return list of publishers
     */
    Collection<Publisher<? extends MessageDefinition>> publishers();

    /**
     * Returns a list of all subscribers created by this instance.
     * @return list of subscribers
     */
    Collection<Subscriber<? extends MessageDefinition>> subscribers();

    /**
     * Returns a list of all timers created by this instance.
     * @return list of timers
     */
    Collection<Timer> timers();

    /**
     * Removes a Publisher instance.
     * @param <T> the type of the messages to be published
     * @param publisher the publisher instance to be removed
     * @return true if the instance was successfully removed, and false otherwise
     */
    <T extends MessageDefinition> boolean removePublisher(final Publisher<T> publisher);

    /**
     * Removes a Subscriber instance.
     * @param <T> the type of the messages to be subscribed
     * @param subscriber the subscriber instance to be removed
     * @return true if the instance was successfully removed, and false otherwise
     */
    <T extends MessageDefinition> boolean removeSubscriber(final Subscriber<T> subscriber);

}
