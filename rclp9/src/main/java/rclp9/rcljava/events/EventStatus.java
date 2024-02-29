package rclp9.rcljava.events;

public interface EventStatus {
    long allocateRCLStatusEvent();
    void deallocateRCLStatusEvent(long handle);
    void fromRCLEvent(long handle);
}
