package rclp9.rcljava.executor;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.event.EventHandler;
import rclp9.rcljava.event.EventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.subscriber.Subscriber;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.util.JNIUtils;

public final class SingleThreadedExecutor implements Executor {
    private static final Logger logger = Logger.getLogger(SingleThreadedExecutor.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(SingleThreadedExecutor.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private final List<ComposableNode> nodes = new ArrayList<>();
    private final List<Map.Entry<Long, Timer>> timerHandlers = new ArrayList<>();
    private final List<Map.Entry<Long, Subscriber<? extends MessageDefinition>>> subscriberHandlers = new ArrayList<>();
    private final List<Map.Entry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>> eventHandlers = new ArrayList<>();
    private boolean executed = false;

    @Override
    public final void addNode(final ComposableNode node) {
        this.nodes.add(node);
    }

    @Override
    public final void removeNode(final ComposableNode node) {
        this.nodes.remove(node);
    }

    @Override
    public final void spinOnce() {
        this.spinOnce(-1);
    }

    @Override
    public final void spinOnce(final long timeout) {
        if(!executeExecutables()){
            waitForWork(timeout);
            executeExecutables();
        }
    }

    @Override
    public final void spin() {
        while (RCLJava.isReady()) {
            this.spinOnce();
        }
    }

    private boolean executeExecutables() {
        executed = false;

        for (var entry : this.timerHandlers) {
            final Optional<Timer> timer = Optional.ofNullable(entry.getValue());
            timer.ifPresent((t) -> {
                if(t.isReady()){
                    entry.setValue(null);
                    
                    t.callTimer();
                    t.executeCallback();
                    this.timerHandlers.remove(new AbstractMap.SimpleEntry<Long, Timer>(t.handle(), t));
                    executed = true;
                }
            });
        }

        for (var entry : this.subscriberHandlers) {
            Optional<Subscriber<? extends MessageDefinition>> subscriber = Optional.ofNullable(entry.getValue());
            subscriber.ifPresent((s) -> {
                entry.setValue(null);
                executeSubscriberCallback(s);
                executed = true;
            });
        }

        for (var entry : this.eventHandlers) {
            Optional<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandler = Optional.ofNullable(entry.getValue());
            eventHandler.ifPresent((h) ->{
                entry.setValue(null);
                
                h.executeCallback();
                this.eventHandlers.remove(
                    new AbstractMap.SimpleEntry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>(h.handle(), h)
                );
                executed = true;
            });
        }

        return executed;
    }

    private <T extends MessageDefinition> void executeSubscriberCallback(Subscriber<T> subscriber){
        Optional<T> message = Optional.ofNullable(nativeTake((long) subscriber.handle(), subscriber.messageType()));
        message.ifPresent((m) -> {
            subscriber.executeCallback(m);
        });
        this.subscriberHandlers.remove(new AbstractMap.SimpleEntry<Long, Subscriber<T>>(subscriber.handle(), subscriber));
    }

    private void waitForWork(final long timeout) {
        this.timerHandlers.clear();
        this.subscriberHandlers.clear();
        this.eventHandlers.clear();

        for (var composableNode : this.nodes) {
            for (var subscriber : composableNode.node().subscribers()) {
                this.subscriberHandlers
                        .add(new AbstractMap.SimpleEntry<Long, Subscriber<? extends MessageDefinition>>(
                                subscriber.handle(), subscriber));

                for (var eventHandler : subscriber.eventHandlers()) {
                    this.eventHandlers.add(
                            new AbstractMap.SimpleEntry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>(
                                    eventHandler.handle(), eventHandler));
                }
            }

            for (var publisher : composableNode.node().publishers()) {
                for (var eventHandler : publisher.eventHandlers()) {
                    this.eventHandlers.add(
                            new AbstractMap.SimpleEntry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>(
                                    eventHandler.handle(), eventHandler));
                }
            }

            for (var timer : composableNode.node().timers()) {
                this.timerHandlers.add(new AbstractMap.SimpleEntry<Long, Timer>(timer.handle(), timer));
            }
        }

        int subscribersSize = 0;
        int timersSize = 0;
        int eventsSize = this.eventHandlers.size();

        for (var composableNode : this.nodes) {
            subscribersSize += composableNode.node().subscribers().size();
            timersSize += composableNode.node().timers().size();
        }

        if (subscribersSize == 0 && timersSize == 0) {
            return;
        }

        long waitSetHandle = nativeGetZeroInitializedWaitSet();
        long contextHandle = RCLJava.defaultContext().handle();
        nativeWaitSetInit(waitSetHandle, contextHandle, subscribersSize, 0, timersSize, 0, 0,
                eventsSize);
        nativeWaitSetClear(waitSetHandle);

        for (var entry : this.subscriberHandlers) {
            nativeWaitSetAddSubscriber(waitSetHandle, entry.getKey());
        }

        for (var entry : this.timerHandlers) {
            nativeWaitSetAddTimer(waitSetHandle, entry.getKey());
        }

        for (var entry : this.eventHandlers) {
            nativeWaitSetAddEvent(waitSetHandle, entry.getKey());
        }

        nativeWait(waitSetHandle, timeout);

        for (int i = 0; i < this.subscriberHandlers.size(); ++i) {
            if (!nativeWaitSetSubscriberIsReady(waitSetHandle, i)) {
                this.subscriberHandlers.get(i).setValue(null);
            }
        }

        for (int i = 0; i < this.timerHandlers.size(); ++i) {
            if (!nativeWaitSetTimerIsReady(waitSetHandle, i)) {
                this.timerHandlers.get(i).setValue(null);
            }
        }

        for (int i = 0; i < this.eventHandlers.size(); ++i) {
            if (!nativeWaitSetEventIsReady(waitSetHandle, i)) {
                this.eventHandlers.get(i).setValue(null);
            }
        }

        var subscriberIterator = this.subscriberHandlers.iterator();
        while (subscriberIterator.hasNext()) {
            var entry = subscriberIterator.next();
            Optional<Subscriber<? extends MessageDefinition>> subscriber = Optional.ofNullable(entry.getValue());
            if (subscriber.isEmpty()) {
                subscriberIterator.remove();
            }
        }

        var timerIterator = this.timerHandlers.iterator();
        while (timerIterator.hasNext()) {
            var entry = timerIterator.next();
            Optional<Timer> timer = Optional.ofNullable(entry.getValue());
            if (timer.isEmpty()) {
                timerIterator.remove();
            }
        }

        var eventIterator = this.eventHandlers.iterator();
        while (eventIterator.hasNext()) {
            var entry = eventIterator.next();
            Optional<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandler = Optional
                    .ofNullable(entry.getValue());
            if (eventHandler.isEmpty()) {
                eventIterator.remove();
            }
        }

        nativeDisposeWaitSet(waitSetHandle);
    }

    private static native void nativeDisposeWaitSet(long waitSetHandle);

    private static native long nativeGetZeroInitializedWaitSet();

    private static native <T extends MessageDefinition> T nativeTake(
            long subscriptionHandle, Class<T> messageType);

    private static native void nativeWait(long waitSetHandle, long timeout);

    private static native void nativeWaitSetAddEvent(long waitSetHandle, long eventHandle);

    private static native void nativeWaitSetAddSubscriber(long waitSetHandle, long subscriptionHandle);

    private static native void nativeWaitSetAddTimer(long waitSetHandle, long timerHandle);

    private static native void nativeWaitSetClear(long waitSetHandle);

    private static native boolean nativeWaitSetEventIsReady(long waitSetHandle, long index);

    private static native void nativeWaitSetInit(
            long waitSetHandle, long contextHandle, int numberOfSubscriptions,
            int numberOfGuardConditions, int numberOfTimers, int numberOfClients,
            int numberOfServices, int numberOfEvents);

    private static native boolean nativeWaitSetSubscriberIsReady(long waitSetHandle, long index);

    private static native boolean nativeWaitSetTimerIsReady(long waitSetHandle, long index);
}
