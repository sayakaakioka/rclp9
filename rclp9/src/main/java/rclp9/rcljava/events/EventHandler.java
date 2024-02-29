package rclp9.rcljava.events;

import java.lang.ref.WeakReference;

import rclp9.rcljava.interfaces.Disposable;

public interface EventHandler<U extends EventStatus, V extends Disposable> extends Disposable {
    WeakReference<V> getParentReference();

    void executeCallback();
}
