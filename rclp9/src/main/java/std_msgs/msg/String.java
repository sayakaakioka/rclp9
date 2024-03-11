package std_msgs.msg;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.util.JNIUtils;

public final class String implements MessageDefinition {
    private static final Logger logger = Logger.getLogger(String.class.getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(String.class);
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private java.lang.String data = "";

    public java.lang.String data() {
        return this.data;
    }

    public final void data(final java.lang.String data) {
        this.data = data;
    }

    @Override
    public final long getFromJavaConverterInstance() {
        return String.getFromJavaConverter();
    }

    @Override
    public final long getToJavaConverterInstance() {
        return String.getToJavaConverter();
    }

    @Override
    public final long getTypeSupportInstance() {
        return String.getTypeSupport();
    }

    @Override
    public final long getDestructorInstance() {
        return String.getDestructor();
    }

    @Override
    public final java.lang.String toString() {
        return this.data();
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();

}
