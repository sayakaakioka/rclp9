package rclp9.rcljava;

import java.time.Duration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.contexts.ContextImpl;
import rclp9.rcljava.executors.SingleThreadedExecutor;
import rclp9.rcljava.interfaces.Context;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.node.NodeImpl;
import rclp9.rcljava.utils.JNIUtils;

public final class RCLJava {
    private static final Logger logger = Logger.getLogger(RCLJava.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(RCLJava.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private static ContextImpl defaultContext;
    private static Collection<Node> nodes = new LinkedBlockingQueue<Node>();;
    private static Collection<Context> contexts = new LinkedBlockingQueue<Context>();
    private static SingleThreadedExecutor globalExecutor = null;

    public static long convertQoSProfileToHandle() {
        var deadline = Duration.ofSeconds(0, 0);
        return nativeConvertQoSProfileToHandle(2, 0, 0, 0,
                deadline.getSeconds(), deadline.getNano(), deadline.getSeconds(), deadline.getNano(),
                0, deadline.getSeconds(), deadline.getNano(), false);
    }

    public static Node createNode(final String nodeName) {
        return createNode(nodeName, "", RCLJava.getDefaultContext());
    }

    public static void disposeQoSProfile(final long qosProfileHandle) {
        nativeDisposeQoSProfile(qosProfileHandle);
    }

    public static synchronized ContextImpl getDefaultContext() {
        Optional<ContextImpl> context = Optional.ofNullable(RCLJava.defaultContext);
        RCLJava.defaultContext = context.orElseGet(() -> new ContextImpl(nativeCreateContextHandle()));
        return RCLJava.defaultContext;
    }

    public static synchronized void shutdown() {
        cleanup();
        if (RCLJava.defaultContext != null) {
            RCLJava.defaultContext.dispose();
            RCLJava.defaultContext = null;
        }
    }

    public static void spin(final ComposableNode composableNode) {
        getGlobalExecutor().addNode(composableNode);
        getGlobalExecutor().spin();
        getGlobalExecutor().removeNode(composableNode);
    }

    public static void spinOnce(final ComposableNode composableNode) {
        getGlobalExecutor().addNode(composableNode);
        getGlobalExecutor().spinOnce();
        getGlobalExecutor().removeNode(composableNode);
    }

    /**
     * Initialize RCLJava.
     */
    public static synchronized void init() {
        logger.info("start initializing the environment\n");
        if (RCLJava.getDefaultContext().isValid()) {
            logger.info("Default context is valid.");
            return;
        }

        getDefaultContext().init();

        var str = nativeGetRMWIdentifier();
        logger.info(String.format("RMW implementation is %s", str));
    }

    public static boolean isReady() {
        return RCLJava.getDefaultContext().isValid();
    }

    private static void cleanup() {
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
        var nodeHandle = nativeCreateNodeHandle(nodeName, namespace, context.getHandle(), new ArrayList<String>());
        var node = new NodeImpl(nodeHandle, context, false);
        nodes.add(node);
        return node;
    }

    private static SingleThreadedExecutor getGlobalExecutor() {
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
