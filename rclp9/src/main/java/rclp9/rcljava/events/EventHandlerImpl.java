package rclp9.rcljava.events;

import java.lang.ref.WeakReference;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.consumers.Consumer;
import rclp9.rcljava.interfaces.Disposable;
import rclp9.rcljava.utils.JNIUtils;

public class EventHandlerImpl<U extends EventStatus, V extends Disposable> implements EventHandler<U, V> {
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
        this.parentReference = parentReference;
        this.handle = handle;
        this.eventStatusFactory = eventStatusFactory;
        this.callback = callback;
        this.disposeCallback = disposeCallback;
    }

    @Override
    public void dispose() {
        if (this.handle == 0) {
            return;
        }

        this.disposeCallback.accept(this);
        nativeDispose(this.handle);
        this.handle = 0;
    }

    @Override
    public final long getHandle() {
        return this.handle;
    }

    @Override
    public final WeakReference<V> getParentReference() {
        return this.parentReference;
    }

    @Override
    public void executeCallback() {
        Optional<U> eventStatus = Optional.ofNullable(eventStatusFactory.get());
        eventStatus.ifPresent((s) -> {
            long nativeEventStatusHandle = s.allocateRCLStatusEvent();
            nativeTake(this.handle, nativeEventStatusHandle);
            s.fromRCLEvent(nativeEventStatusHandle);
            s.deallocateRCLStatusEvent(nativeEventStatusHandle);
            this.callback.accept(s);
        });
    }

    private static native void nativeDispose(long Handle);

    private static native void nativeTake(long event_handle, long event_status_handle);
}
