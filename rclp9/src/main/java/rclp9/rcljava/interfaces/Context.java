package rclp9.rcljava.interfaces;

public interface Context extends Disposable {
    /**
     * initialize the context
     */
    void init();

    /**
     * shutdown the context
     */
    void shutdown();

    /**
     * @return true if the context is valid
     */
    boolean isValid();
}
