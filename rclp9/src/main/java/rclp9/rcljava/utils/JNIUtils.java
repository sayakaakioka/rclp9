package rclp9.rcljava.utils;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

public class JNIUtils {
    private static final Logger logger = Logger.getLogger(JNIUtils.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    public static void loadImplementation(Class<?> cls) {
        var libraryName = normalizeClassName(cls);
        logger.info("Loading library: " + libraryName);
        System.loadLibrary(libraryName);
    }

    public static void loadTypesupport(Class<?> cls) {
        var libraryName = normalizeClassName(cls);
        logger.info("Loading typesupport: " + libraryName);
        System.loadLibrary(libraryName);
    }

    private static String normalizeClassName(Class<?> cls) {
        return cls.getCanonicalName().replaceAll("\\.", "_");
    }
}
