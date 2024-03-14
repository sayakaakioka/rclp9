package rclp9.rcljava.executor;

import rclp9.rcljava.node.ComposableNode;

/**
 * This interface defines the APIs for executors. 
 * An executor is responsible for executing publishers, subscribers,
 * timers, and any other events.
 */
public interface Executor {
    /**
     * Puts the specified node under control.
     * @param node a node instance
     */
    public void addNode(ComposableNode node);

    /**
     * Releases the specified node from control.
     * @param node a node instance
     */
    public void removeNode(ComposableNode node);

    /**
     * Sweeps and executes any executables in the list only once.
     */
    public void spinOnce();

    /**
     * Sweeps and executes any executables in the list only once, 
     * with the specified timeout.
     * @param timeout waiting period for the timeout
     */
    public void spinOnce(long timeout);

    /**
     * Sweeps and executes any executables in the list repeatedly.
     */
    public void spin();
}
