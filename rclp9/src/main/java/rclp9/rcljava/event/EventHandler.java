package rclp9.rcljava.event;

import java.lang.ref.WeakReference;

import rclp9.rcljava.interfaces.Disposable;

public interface EventHandler<U extends EventStatus, V extends Disposable> extends Disposable {
    WeakReference<V> parentReference();

    void executeCallback();
}
