package rclp9.rcljava.executors;

import rclp9.rcljava.node.ComposableNode;

public interface Executor {
    public void addNode(ComposableNode node);

    public void removeNode(ComposableNode node);

    public void spinOnce();

    public void spinOnce(long timeout);

    public void spinSome();

    public void spinSome(long maxDurationNs);

    public void spinAll(long maxDurationNs);

    public void spin();
}
