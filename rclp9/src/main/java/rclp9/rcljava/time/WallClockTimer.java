package rclp9.rcljava.time;

public interface WallClockTimer extends Timer {
    long timerPeriodNS();

    void timerPeriodNS(long period);

    boolean isCancelled();

    void cancel();

    void reset();

    long timeSinceLastCall();

    long timeUntilNextCall();
}
