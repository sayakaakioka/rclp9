package rclp9.rcljava.executor;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.subscriber.Subscriber;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class privdes an implementation of the Executor interface.
 * This executor sequentially processes timers and subscribers.
 */
public final class SingleThreadedExecutor implements Executor {
    private static final Logger logger = Logger.getLogger(new Object(){}.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object(){}.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private final List<ComposableNode> nodes = new ArrayList<>();
    private final List<Map.Entry<Long, Timer>> timerHandles = new ArrayList<>();
    // TODO: Consider that the message type may be a mix of several types.
    private final List<Map.Entry<Long, Subscriber<? extends MessageDefinition>>> subscriberHandles = new ArrayList<>();

    /**
     * Constructor performs no operations.
     */
    public SingleThreadedExecutor(){}

    /**
     * {@inheritDoc}
     */
    @Override
    public final void addNode(final ComposableNode node) {
        this.nodes.add(node);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void removeNode(final ComposableNode node) {
        this.nodes.remove(node);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void spinOnce() {
        this.spinOnce(-1);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void spinOnce(final long timeout) {
        if(!executeExecutables()){
            waitForWork(timeout);
            executeExecutables();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final void spin() {
        while (RCLJava.isReady()) {
            this.spinOnce();
        }
    }

    private boolean executeExecutables() {
        boolean executed = false;

        Iterator<Map.Entry<Long, Timer>> timerItr = this.timerHandles.iterator();
        while(timerItr.hasNext()) {
            final Optional<Timer> timer = Optional.ofNullable(timerItr.next().getValue());
            if(timer.isPresent() && timer.get().isReady()){
                var t = timer.get();
                t.callTimer();
                t.executeCallback();
                timerItr.remove();
                executed = true;
            }
        }

        Iterator<Map.Entry<Long, Subscriber<? extends MessageDefinition>>> subscriberItr = this.subscriberHandles.iterator();
        while(subscriberItr.hasNext()){
            Optional<Subscriber<? extends MessageDefinition>> subscriber = Optional.ofNullable(subscriberItr.next().getValue());
            if(subscriber.isPresent()){
                executeSubscriberCallback(subscriber.get());
                subscriberItr.remove();
                executed = true;
            }
        }
        return executed;
    }

    private <T extends MessageDefinition> void executeSubscriberCallback(Subscriber<T> subscriber){
        Optional<T> message = Optional.ofNullable(nativeTake((long) subscriber.handle(), subscriber.messageType()));
        message.ifPresent((m) -> {
            subscriber.executeCallback(m);
        });
        //this.subscriberHandles.remove(new AbstractMap.SimpleEntry<Long, Subscriber<T>>(subscriber.handle(), subscriber));
    }

    private void waitForWork(final long timeout) {
        this.timerHandles.clear();
        this.subscriberHandles.clear();

        int subscribersSize = 0;
        int timersSize = 0;
        for (var composableNode : this.nodes) {
            for (var subscriber : composableNode.node().subscribers()) {
                this.subscriberHandles
                        .add(new AbstractMap.SimpleEntry<Long, Subscriber<? extends MessageDefinition>>(
                                subscriber.handle(), subscriber));
                subscribersSize++;
            }

            for (var timer : composableNode.node().timers()) {
                this.timerHandles.add(new AbstractMap.SimpleEntry<Long, Timer>(timer.handle(), timer));
                timersSize++;
            }
        }

        if (subscribersSize == 0 && timersSize == 0) {
            return;
        }

        long waitSetHandle = nativeGetZeroInitializedWaitSet();
        long contextHandle = RCLJava.defaultContext().handle();
        nativeWaitSetInit(waitSetHandle, contextHandle, subscribersSize, 0, timersSize, 0, 0,
                0);
        nativeWaitSetClear(waitSetHandle);

        for (var entry : this.subscriberHandles) {
            nativeWaitSetAddSubscriber(waitSetHandle, entry.getKey());
        }

        for (var entry : this.timerHandles) {
            nativeWaitSetAddTimer(waitSetHandle, entry.getKey());
        }

        nativeWait(waitSetHandle, timeout);

        for (int i = 0; i < this.subscriberHandles.size(); ++i) {
            if (!nativeWaitSetSubscriberIsReady(waitSetHandle, i)) {
                this.subscriberHandles.get(i).setValue(null);
            }
        }

        for (int i = 0; i < this.timerHandles.size(); ++i) {
            if (!nativeWaitSetTimerIsReady(waitSetHandle, i)) {
                this.timerHandles.get(i).setValue(null);
            }
        }

        var subscriberItr = this.subscriberHandles.iterator();
        while (subscriberItr.hasNext()) {
            var entry = subscriberItr.next();
            Optional<Subscriber<? extends MessageDefinition>> subscriber = Optional.ofNullable(entry.getValue());
            if (subscriber.isEmpty()) {
                subscriberItr.remove();
            }
        }

        var timerItr = this.timerHandles.iterator();
        while (timerItr.hasNext()) {
            var entry = timerItr.next();
            Optional<Timer> timer = Optional.ofNullable(entry.getValue());
            if (timer.isEmpty()) {
                timerItr.remove();
            }
        }

        nativeDisposeWaitSet(waitSetHandle);
    }

    private static native void nativeDisposeWaitSet(long waitSetHandle);

    private static native long nativeGetZeroInitializedWaitSet();

    private static native <T extends MessageDefinition> T nativeTake(
            long subscriptionHandle, Class<T> messageType);

    private static native void nativeWait(long waitSetHandle, long timeout);

    private static native void nativeWaitSetAddSubscriber(long waitSetHandle, long subscriptionHandle);

    private static native void nativeWaitSetAddTimer(long waitSetHandle, long timerHandle);

    private static native void nativeWaitSetClear(long waitSetHandle);

    private static native void nativeWaitSetInit(
            long waitSetHandle, long contextHandle, int numberOfSubscriptions,
            int numberOfGuardConditions, int numberOfTimers, int numberOfClients,
            int numberOfServices, int numberOfEvents);

    private static native boolean nativeWaitSetSubscriberIsReady(long waitSetHandle, long index);

    private static native boolean nativeWaitSetTimerIsReady(long waitSetHandle, long index);
}
