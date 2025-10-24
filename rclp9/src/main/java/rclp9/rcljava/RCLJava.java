package rclp9.rcljava;

import java.time.Duration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.context.Context;
import rclp9.rcljava.context.ContextImpl;
import rclp9.rcljava.executor.SingleThreadedExecutor;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.node.NodeImpl;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class provides the APIs for interacting with RCL.
 */
public final class RCLJava {
    private static final Logger logger = Logger.getLogger(new Object(){}.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object(){}.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe(logger.getName() + ": Failed to load native library.");
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * This class is always static.
     */
    private RCLJava() {
        throw new AssertionError();
    }

    private static ContextImpl defaultContext;
    private final static Collection<Node> nodes = new LinkedBlockingQueue<>();
    private final static Collection<Context> contexts = new LinkedBlockingQueue<>();
    private static SingleThreadedExecutor globalExecutor = null;

    /**
     * Gets a ROS2 handle configured according to the QoS profile.
     * @return ROS2 handle
     */
    public final static long convertQoSProfileToHandle() {
        var deadline = Duration.ofSeconds(0, 0);
        return nativeConvertQoSProfileToHandle(2, 0, 0, 0,
                deadline.getSeconds(), deadline.getNano(), deadline.getSeconds(), deadline.getNano(),
                0, deadline.getSeconds(), deadline.getNano(), false);
    }

    /**
     * Creates a node instance.
     * @param nodeName name of the node
     * @return a node instance
     */
    public final static Node createNode(final String nodeName) {
        var context = RCLJava.defaultContext();
        return createNode(nodeName, "", context);
    }

    /**
     * Gets a default context of ROS2.
     * @return the default context
     */
    public final static synchronized ContextImpl defaultContext() {
        if (RCLJava.defaultContext == null) {
            long handle = nativeCreateContextHandle();
            RCLJava.defaultContext = new ContextImpl(handle);
        }
        return RCLJava.defaultContext;
    }

    /**
     * Releases the resources associated with the specified QoS profile.
     * @param qosProfileHandle the ROS2 handle associated with QoS profile
     */
    public final static void disposeQoSProfile(final long qosProfileHandle) {
        nativeDisposeQoSProfile(qosProfileHandle);
    }


    /**
     * Initializes RCLJava. 
     * This method must be invoked before any other methods are called.
     */
    public static final synchronized void init() {
        logger.info("start initializing the environment\n");
        if (RCLJava.defaultContext().isValid()) {
            logger.info("Default context is valid.");
            return;
        }

        defaultContext().init();

        var str = nativeGetRMWIdentifier();
        logger.info(String.format("RMW implementation is %s", str));
    }

    /**
     * Checks whether the default context is ready or not.
     * @return true if it is ready, and false otherwise.
     */
    public static final boolean isReady() {
        return RCLJava.defaultContext().isValid();
    }

    /**
     * Spins the specified composable node forever.
     * @param composableNode the node to spin
     */
    public static final void spin(final ComposableNode composableNode) {
        globalExecutor().addNode(composableNode);
        globalExecutor().spin();
        globalExecutor().removeNode(composableNode);
    }

    /**
     * Spins the specified composable node once.
     * @param composableNode the node to spin
     */
    public static final void spinOnce(final ComposableNode composableNode) {
        globalExecutor().addNode(composableNode);
        globalExecutor().spinOnce();
        globalExecutor().removeNode(composableNode);
    }

    /**
     * Shuts down the default context.
     */
    public final static synchronized void shutdown() {
        cleanup();
        Optional<ContextImpl> context = Optional.ofNullable(RCLJava.defaultContext());
        context.ifPresent((c) -> {
            RCLJava.defaultContext().dispose();
            RCLJava.defaultContext = null;
        });
    }

    private static final void cleanup() {
        for (var node : nodes) {
            node.dispose();
        }
        nodes.clear();

        for (var context : contexts) {
            context.dispose();
        }
        contexts.clear();
    }

    private static Node createNode(final String nodeName, final String namespace, final Context context) {
        var nodeHandle = nativeCreateNodeHandle(nodeName, namespace, context.handle(), new ArrayList<String>());
        var node = new NodeImpl(nodeHandle, context);
        nodes.add(node);
        return node;
    }

    private static SingleThreadedExecutor globalExecutor() {
        synchronized (RCLJava.class) {
            Optional<SingleThreadedExecutor> executor = Optional.ofNullable(globalExecutor);
            globalExecutor = executor.orElseGet(() -> new SingleThreadedExecutor());
            return globalExecutor;
        }
    }

    private static native long nativeConvertQoSProfileToHandle(int history, int depth, int reliability,
            int durability, long deadlineSec, int deadlineNanos,
            long lifespanSec, int lifespanNanos, int liveliness, long livelinessLeaseSec,
            int livelinessLeaseNanos, boolean avoidROSNamespaceConventions);

    private static native long nativeCreateContextHandle();

    private static native long nativeCreateNodeHandle(String nodeName, String namespace, long contextHandle,
            ArrayList<String> arguments);

    private static native void nativeDisposeQoSProfile(long qosProfileHandle);

    private static native String nativeGetRMWIdentifier();

}
