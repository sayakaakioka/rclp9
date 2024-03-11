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
import rclp9.rcljava.time.WallClockTimer;

/**
 * This class is comparable with rcl_node_t in ROS2.
 * A node should be created via @{link RCLJava#createNode(String).}
 */
public interface Node extends Disposable {
    /**
     * Create a Publisher.
     *
     * @param <T>         The type of the messages to be published.
     * @param messageType The class of the messages to be published.
     * @param topic       The topic to which the messages will be published.
     * @return A {@link Publisher}.
     */
    <T extends MessageDefinition> Publisher<T> createPublisher(final Class<T> messageType, final String topic);

    /**
     * Create a Subscriber.
     *
     * @param <T>         The type of the messages to be subscribed.
     * @param messageType The class of the messages to be subscribed.
     * @param topic       The topic where the mssages will be subscribed.
     * @param callback    The callback function to be called on a message arrival.
     * @return A {@link Subscriber}
     */
    <T extends MessageDefinition> Subscriber<T> createSubscriber(final Class<T> messageType, final String topic,
            final Consumer<T> callback);

    /**
     * Create a WallClockTimer
     *
     * @param period   The time interval.
     * @param unit     The unit for period.
     * @param callback The callback function to be called.
     * @return A {@link WallClockTimer}.
     */
    WallClockTimer createWallClockTimer(final long period, final TimeUnit unit, final Callback callback);

    /**
     * @return All the {@link Publisher}s this instance has created.
     */
    Collection<Publisher<? extends MessageDefinition>> publishers();

    /**
     * @return All the {@link Subscriber}s this instance has created.
     */
    Collection<Subscriber<? extends MessageDefinition>> subscribers();

    /**
     * @return All the {@link Timer}s this instance has created.
     */
    Collection<Timer> timers();

    /**
     * Remove a Publisher.
     *
     * @param publisher The object to be removed.
     * @return true if the Publisher was successfully removed.
     */
    <T extends MessageDefinition> boolean removePublisher(final Publisher<T> publisher);

    /**
     * Remove a Subscriber.
     *
     * @param subscriber The object to be removed.
     * @return true if the Subscriber was successfully removed.
     */
    <T extends MessageDefinition> boolean removeSubscriber(final Subscriber<T> subscriber);

}
