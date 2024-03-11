package rclp9.rcljava.util;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

public final class JNIUtils {
    private static final Logger logger = Logger.getLogger(JNIUtils.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    /**
     * This class is always static.
     */
    private JNIUtils() {
        throw new AssertionError();
    }

    public static final void loadImplementation(Class<?> cls) {
        var libraryName = normalizeClassName(cls);
        logger.info("Loading library: " + libraryName);
        System.loadLibrary(libraryName);
    }

    public static final void loadTypesupport(Class<?> cls) {
        var libraryName = normalizeClassName(cls);
        logger.info("Loading typesupport: " + libraryName);
        System.loadLibrary(libraryName);
    }

    private static String normalizeClassName(Class<?> cls) {
        return cls.getCanonicalName().replaceAll("\\.", "_");
    }
}
