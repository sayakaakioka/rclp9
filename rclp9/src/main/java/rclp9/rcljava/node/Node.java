package rclp9.rcljava.node;

import java.util.Collection;
import java.util.concurrent.TimeUnit;

import rclp9.rcljava.interfaces.Callback;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.publisher.Publisher;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.time.WallClockTimer;

/**
 * This class is comparable with rcl_node_t in ROS2.
 * A node should be created via @{link RCLJava#createNode(String).}
 */
public interface Node extends Disposable {
    /**
     * @return all the {@link Publisher}s this instance has created.
     */
    Collection<Publisher<? extends MessageDefinition>> getPublishers();

    /**
     * @return all the {@link Timer}s this instance has created.
     */
    Collection<Timer> getTimers();

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
     * Create a Publisher.
     *
     * @param <T>         The type of the messages to be published.
     * @param messageType The class of the messages to be published.
     * @param topic       The topic to which the messages will be published.
     * @return A {@link Publisher}.
     */
    <T extends MessageDefinition> Publisher<T> createPublisher(final Class<T> messageType, final String topic);

    /**
     * Remove a Publisher.
     *
     * @param publisher The object to be removed.
     * @return true if the publisher was successfully removed.
     */
    boolean removePublisher(final Publisher<? extends MessageDefinition> publisher);

}
