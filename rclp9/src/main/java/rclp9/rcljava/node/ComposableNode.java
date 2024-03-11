package rclp9.rcljava.node;

public interface ComposableNode {
    Node node();

    /**
     * Return the information of this object.
     *
     * @return Object information in String.
     */
    String toString();
}
