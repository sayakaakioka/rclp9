package std_msgs.msg;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class defines the structure for messages of String.
 */
public final class String implements MessageDefinition {
    private static final Logger logger = Logger.getLogger(new Object() {}.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object() {
            }.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    /**
     * data in its raw
     */
    public java.lang.String data = "";

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return new HashCodeBuilder().append(data).toHashCode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof String)) {
            return false;
        }

        if (this == obj) {
            return true;
        }

        String s = (String) obj;
        return new EqualsBuilder().append(data, s.data).isEquals();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getFromJavaConverterInstance() {
        return String.getFromJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getToJavaConverterInstance() {
        return String.getToJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getTypeSupportInstance() {
        return String.getTypeSupport();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getDestructorInstance() {
        return String.getDestructor();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final java.lang.String toString() {
        return this.data;
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();

}
