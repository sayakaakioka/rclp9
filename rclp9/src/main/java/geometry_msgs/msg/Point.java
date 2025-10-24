package geometry_msgs.msg;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class defines the structure for messages of Point.
 */
public class Point implements MessageDefinition {
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
            logger.severe(logger.getName() + ": Failed to load native library.");
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * x value in its raw
     */
    public double x;

    /**
     * y value in its raw
     */
    public double y;

    /**
     * z value in its raw
     */
    public double z;

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return new HashCodeBuilder().append(x).append(y).append(z).toHashCode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(final Object obj) {
        if (!(obj instanceof Point)) {
            return false;
        }

        if (this == obj) {
            return true;
        }

        Point p = (Point) obj;
        return new EqualsBuilder().append(x, p.x).append(y, p.y).append(z, p.z).isEquals();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getFromJavaConverterInstance() {
        return Point.getFromJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getToJavaConverterInstance() {
        return Point.getToJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getTypeSupportInstance() {
        return Point.getTypeSupport();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getDestructorInstance() {
        return Point.getDestructor();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final java.lang.String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();

}
