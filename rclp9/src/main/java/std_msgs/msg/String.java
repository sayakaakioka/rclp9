package std_msgs.msg;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.utils.JNIUtils;

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

    @Override
    public long getFromJavaConverterInstance() {
        return String.getFromJavaConverter();
    }

    @Override
    public long getToJavaConverterInstance() {
        throw new UnsupportedOperationException("Unimplemented method 'getToJavaConverterInstance()'");
        // return String.getToJavaConverter();
    }

    @Override
    public long getTypeSupportInstance() {
        return String.getTypeSupport();
    }

    public java.lang.String getData() {
        return this.data;
    }

    @Override
    public long getDestructorInstance() {
        return String.getDestructor();
    }

    public void setData(final java.lang.String data) {
        this.data = data;
    }

    private static native long getDestructor();

    private static native long getFromJavaConverter();

    private static native long getToJavaConverter();

    private static native long getTypeSupport();

}
