package rclp9.rcljava.executors;

import rclp9.rcljava.RCLJava;
import rclp9.rcljava.node.ComposableNode;

public class SingleThreadedExecutor implements Executor {
    private BaseExecutor baseExecutor = new BaseExecutor();

    @Override
    public void addNode(ComposableNode node) {
        this.baseExecutor.addNode(node);
    }

    @Override
    public void removeNode(ComposableNode node) {
        this.baseExecutor.removeNode(node);
    }

    @Override
    public void spinOnce() {
        this.spinOnce(-1);
    }

    @Override
    public void spinOnce(long timeout) {
        this.baseExecutor.spinOnce(timeout);
    }

    @Override
    public void spinSome() {
        this.spinSome(0);
    }

    @Override
    public void spinSome(long maxDurationNs) {
        this.baseExecutor.spinSome(maxDurationNs);
    }

    @Override
    public void spinAll(long maxDurationNs) {
        this.baseExecutor.spinAll(maxDurationNs);
    }

    @Override
    public void spin() {
        while (RCLJava.isReady()) {
            this.spinOnce();
        }
    }

}
