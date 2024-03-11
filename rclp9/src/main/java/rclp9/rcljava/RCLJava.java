package rclp9.rcljava;

import java.time.Duration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.context.ContextImpl;
import rclp9.rcljava.executor.SingleThreadedExecutor;
import rclp9.rcljava.interfaces.Context;
import rclp9.rcljava.node.ComposableNode;
import rclp9.rcljava.node.Node;
import rclp9.rcljava.node.NodeImpl;
import rclp9.rcljava.util.JNIUtils;

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

    private RCLJava() {
        throw new AssertionError();
    }

    private static ContextImpl defaultContext;
    private final static Collection<Node> nodes = new LinkedBlockingQueue<>();
    private final static Collection<Context> contexts = new LinkedBlockingQueue<>();
    private static SingleThreadedExecutor globalExecutor = null;

    public final static long convertQoSProfileToHandle() {
        var deadline = Duration.ofSeconds(0, 0);
        return nativeConvertQoSProfileToHandle(2, 0, 0, 0,
                deadline.getSeconds(), deadline.getNano(), deadline.getSeconds(), deadline.getNano(),
                0, deadline.getSeconds(), deadline.getNano(), false);
    }

    public final static Node createNode(final String nodeName) {
        var context = RCLJava.defaultContext();
        return createNode(nodeName, "", context);
    }

    public final static synchronized ContextImpl defaultContext() {
        if (RCLJava.defaultContext == null) {
            long handle = nativeCreateContextHandle();
            RCLJava.defaultContext = new ContextImpl(handle);
        }
        return RCLJava.defaultContext;
    }

    public final static void disposeQoSProfile(final long qosProfileHandle) {
        nativeDisposeQoSProfile(qosProfileHandle);
    }


    /**
     * Initialize RCLJava.
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

    public static final boolean isReady() {
        return RCLJava.defaultContext().isValid();
    }

    public static final void spin(final ComposableNode composableNode) {
        globalExecutor().addNode(composableNode);
        globalExecutor().spin();
        globalExecutor().removeNode(composableNode);
    }

    public static final void spinOnce(final ComposableNode composableNode) {
        globalExecutor().addNode(composableNode);
        globalExecutor().spinOnce();
        globalExecutor().removeNode(composableNode);
    }

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
