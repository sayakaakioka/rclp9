package rclp9.rcljava.time;

public interface  WallClockTimer extends Timer {
    long getTimerPeriodNS();

    void setTimerPeriodNS(long period);

    boolean isCancelled();

    void cancel();

    void reset();

    long timeSinceLastCall();

    long timeUntilNextCall();
}
