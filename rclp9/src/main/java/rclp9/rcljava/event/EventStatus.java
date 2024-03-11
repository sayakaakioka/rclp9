package rclp9.rcljava.event;

public interface EventStatus {
    long allocateRCLStatusEvent();

    void deallocateRCLStatusEvent(long handle);

    void fromRCLEvent(long handle);
}
