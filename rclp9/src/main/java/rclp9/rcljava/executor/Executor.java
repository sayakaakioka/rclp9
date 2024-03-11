package rclp9.rcljava.executor;

import rclp9.rcljava.node.ComposableNode;

public interface Executor {
    public void addNode(ComposableNode node);

    public void removeNode(ComposableNode node);

    public void spinOnce();

    public void spinOnce(long timeout);

    public void spin();
}
