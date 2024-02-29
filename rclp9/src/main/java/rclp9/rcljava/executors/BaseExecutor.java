package rclp9.rcljava.executors;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.events.EventHandler;
import rclp9.rcljava.events.EventStatus;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.time.Timer;
import rclp9.rcljava.utils.JNIUtils;

public class BaseExecutor {
    private static final Logger logger = Logger.getLogger(BaseExecutor.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(BaseExecutor.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private BlockingQueue<ComposableNode> nodes = new LinkedBlockingQueue<ComposableNode>();
    private List<Map.Entry<Long, Timer>> timerHandles = new ArrayList<Map.Entry<Long, Timer>>();
    private List<Map.Entry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>> eventHandles = new ArrayList<Map.Entry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>>();

    protected void addNode(ComposableNode node) {
        this.nodes.add(node);
    }

    protected void removeNode(ComposableNode node) {
        this.nodes.remove(node);
    }

    protected void spinAll(long maxDurationNs) {
        spinSomeImpl(maxDurationNs, true);
    }

    protected void spinOnce(long timeout) {
        Optional<AnyExecutable> anyExecutable = Optional.ofNullable(getNextExecutable());
        anyExecutable.ifPresentOrElse((ae) -> {
            executeAnyExecutable(ae);
        }, () -> {
            waitForWork(timeout);
            Optional<AnyExecutable> executable = Optional.ofNullable(getNextExecutable());
            executable.ifPresent((e) -> executeAnyExecutable(e));
        });
    }

    protected void spinSome(long maxDurationNs) {
        spinSomeImpl(maxDurationNs, false);
    }

    private void executeAnyExecutable(AnyExecutable anyExecutable) {
        Optional<Timer> timer = Optional.ofNullable(anyExecutable.timer);
        timer.ifPresent((t) -> {
            t.callTimer();
            t.executeCallback();
            timerHandles.remove(
                    timerHandles
                            .remove(anyExecutable.timer.getHandle()));
        });

        Optional<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandler = Optional
                .ofNullable(anyExecutable.eventHandler);
        eventHandler.ifPresent((h) -> {
            h.executeCallback();
            eventHandles.remove(
                    anyExecutable.eventHandler.getHandle());
        });
    }

    private AnyExecutable getNextExecutable() {
        var anyExecutable = new AnyExecutable();

        for (var entry : this.timerHandles) {
            Optional<Timer> timer = Optional.ofNullable(entry.getValue());
            if (timer.isPresent() && timer.get().isReady()) {
                anyExecutable.timer = timer.get();
                entry.setValue(null);
                return anyExecutable;
            }
        }

        for (var entry : this.eventHandles) {
            Optional<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandler = Optional
                    .ofNullable(entry.getValue());

            if (eventHandler.isPresent()) {
                anyExecutable.eventHandler = eventHandler.get();
                entry.setValue(null);
                return anyExecutable;
            }
        }

        return null;
    }

    private boolean maxDurationNotElapsed(long maxDurationNs, long startNs) {
        long nowNs = System.nanoTime();
        if (maxDurationNs == 0) {
            return true;
        } else if (nowNs - startNs < maxDurationNs) {
            return true;
        }
        return false;
    }

    private void spinSomeImpl(long maxDurationNs, boolean exhaustive) {
        long startNs = System.nanoTime();
        boolean workAvailable = false;
        while (RCLJava.isReady() && maxDurationNotElapsed(maxDurationNs, startNs)) {
            if (!workAvailable) {
                waitForWork(0);
            }

            Optional<AnyExecutable> anyExecutable = Optional.ofNullable(getNextExecutable());
            if (anyExecutable.isPresent()) {
                executeAnyExecutable(anyExecutable.get());
            } else {
                if (!workAvailable || !exhaustive) {
                    break;
                }
                workAvailable = false;
            }
        }
    }

    private void waitForWork(long timeout) {
        this.timerHandles.clear();
        this.eventHandles.clear();

        for (var node : this.nodes) {
            for (var publisher : node.getNode().getPublishers()) {
                var eventHandlers = publisher.getEventHandlers();
                for (var eventHandler : eventHandlers) {
                    this.eventHandles.add(
                            new AbstractMap.SimpleEntry<Long, EventHandler<? extends EventStatus, ? extends Disposable>>(
                                    eventHandler.getHandle(), eventHandler));
                }
            }

            for (var timer : node.getNode().getTimers()) {
                this.timerHandles.add(new AbstractMap.SimpleEntry<Long, Timer>(timer.getHandle(), timer));
            }
        }

        int subscriptionsSize = 0;
        int timersSize = 0;
        int clientsSize = 0;
        int servicesSize = 0;
        int eventsSize = this.eventHandles.size();

        for (var node : this.nodes) {
            timersSize += node.getNode().getTimers().size();
        }

        if (subscriptionsSize == 0 && timersSize == 0 && clientsSize == 0 && servicesSize == 0) {
            return;
        }

        long waitSetHandle = nativeGetZeroInitializedWaitSet();
        long contextHandle = RCLJava.getDefaultContext().getHandle();
        nativeWaitSetInit(waitSetHandle, contextHandle, subscriptionsSize, 0, timersSize, clientsSize, servicesSize,
                eventsSize);
        nativeWaitSetClear(waitSetHandle);

        for (var entry : this.timerHandles) {
            nativeWaitSetAddTimer(waitSetHandle, entry.getKey());
        }

        for (var entry : this.eventHandles) {
            nativeWaitSetAddEvent(waitSetHandle, entry.getKey());
        }

        nativeWait(waitSetHandle, timeout);

        for (int i = 0; i < this.timerHandles.size(); ++i) {
            if (!nativeWaitSetTimerIsReady(waitSetHandle, i)) {
                this.timerHandles.get(i).setValue(null);
            }
        }

        for (int i = 0; i < this.eventHandles.size(); ++i) {
            if (!nativeWaitSetEventIsReady(waitSetHandle, i)) {
                this.eventHandles.get(i).setValue(null);
            }
        }

        var timerIterator = this.timerHandles.iterator();
        while (timerIterator.hasNext()) {
            var entry = timerIterator.next();
            Optional<Timer> timer = Optional.ofNullable(entry.getValue());
            timer.ifPresentOrElse((t) -> {
            },
                    () -> {
                        timerIterator.remove();
                    });
        }

        var eventIterator = this.eventHandles.iterator();
        while (eventIterator.hasNext()) {
            var entry = eventIterator.next();
            Optional<EventHandler<? extends EventStatus, ? extends Disposable>> eventHandler = Optional
                    .ofNullable(entry.getValue());
            eventHandler.ifPresentOrElse((h) -> {
            },
                    () -> {
                        eventIterator.remove();
                    });
        }

        nativeDisposeWaitSet(waitSetHandle);
    }

    private static native void nativeDisposeWaitSet(long waitSetHandle);

    private static native long nativeGetZeroInitializedWaitSet();

    private static native void nativeWait(long waitSetHandle, long timeout);

    private static native void nativeWaitSetAddTimer(long waitSetHandle, long timerHandle);

    private static native void nativeWaitSetInit(
            long waitSetHandle, long contextHandle, int numberOfSubscriptions,
            int numberOfGuardConditions, int numberOfTimers, int numberOfClients,
            int numberOfServices, int numberOfEvents);

    private static native void nativeWaitSetClear(long waitSetHandle);

    private static native void nativeWaitSetAddEvent(long waitSetHandle, long eventHandle);

    private static native boolean nativeWaitSetEventIsReady(long waitSetHandle, long index);

    private static native boolean nativeWaitSetTimerIsReady(long waitSetHandle, long index);

}
