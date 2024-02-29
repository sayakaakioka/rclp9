package rclp9.rcljava.node;

import rclp9.rcljava.RCLJava;

public class BaseComposableNode implements ComposableNode {
    static {
        RCLJava.init();
    }

    private final String name;
    protected final Node node;

    public BaseComposableNode(String name) {
        this.name = name;
        this.node = RCLJava.createNode(this.name);
    }

    @Override
    public Node getNode() {
        return node;
    }

}
