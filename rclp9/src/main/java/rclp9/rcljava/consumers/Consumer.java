package rclp9.rcljava.consumers;

public interface Consumer<T> {
    void accept(T input);
}
