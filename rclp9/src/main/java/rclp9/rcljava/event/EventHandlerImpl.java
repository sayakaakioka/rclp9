package rclp9.rcljava.event;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.Objects;

import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.util.JNIUtils;

public final class EventHandlerImpl<U extends EventStatus, V extends Disposable> implements EventHandler<U, V> {
    private static final Logger logger = Logger.getLogger(EventHandlerImpl.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(EventHandlerImpl.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private WeakReference<V> parentReference;
    private long handle;
    private Supplier<U> eventStatusFactory;
    private Consumer<U> callback;
    private Consumer<EventHandler<U, V>> disposeCallback;

    public EventHandlerImpl(final WeakReference<V> parentReference, final long handle,
            final Supplier<U> eventStatusFactory, final Consumer<U> callback,
            final Consumer<EventHandler<U, V>> disposeCallback) {
        this.parentReference = Objects.requireNonNull(parentReference);
        this.handle = Objects.requireNonNull(handle);
        this.eventStatusFactory = Objects.requireNonNull(eventStatusFactory);
        this.callback = Objects.requireNonNull(callback);
        this.disposeCallback = Objects.requireNonNull(disposeCallback);
    }

    @Override
    public final void dispose() {
        if (this.handle == 0) {
            return;
        }

        this.disposeCallback.accept(this);
        nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public final long handle() {
        return this.handle;
    }

    @Override
    public final void executeCallback() {
        Optional<U> eventStatus = Optional.ofNullable(eventStatusFactory.get());
        eventStatus.ifPresent((s) -> {
            long nativeEventStatusHandle = s.allocateRCLStatusEvent();
            nativeTake(this.handle, nativeEventStatusHandle);
            s.fromRCLEvent(nativeEventStatusHandle);
            s.deallocateRCLStatusEvent(nativeEventStatusHandle);
            this.callback.accept(s);
        });
    }

    @Override
    public final WeakReference<V> parentReference() {
        return this.parentReference;
    }

    @Override
    public final String toString() {
        return ("Instance of " + EventHandlerImpl.class.getName() + ", handle = " + handle);
    }

    private static native void nativeDispose(long Handle);

    private static native void nativeTake(long event_handle, long event_status_handle);
}
