package std_msgs.msg;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;

import rclp9.rcljava.interfaces.MessageDefinition;
import rclp9.rcljava.util.JNIUtils;

/**
 * This class defines the structure for messages of Float64.
 */
public class Float64 implements MessageDefinition{
    private static final Logger logger = Logger.getLogger(new Object(){}.getClass().getName());
    {
        logger.addHandler(new ConsoleHandler());
        logger.setLevel(Level.INFO);
    }

    static {
        try {
            JNIUtils.loadImplementation(new Object(){}.getClass().getEnclosingClass());
        } catch (UnsatisfiedLinkError e) {
            logger.severe("Failed to load native library.");
            System.exit(1);
        }
    }

    private double data;

    /**
     * Gets the value of data.
     * @return data in double
     */
    public double data() {
        return this.data;
    }

    /**
     * Sets the value of data.
     * @param data value to be set
     */
    public final void data(final double data) {
        this.data = data;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode(){
        return new HashCodeBuilder().append(data).toHashCode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(final Object obj){
        if(!(obj instanceof Float64)){
            return false;
        }

        if(this == obj){
            return true;
        }

        Float64 f = (Float64)obj;
        return new EqualsBuilder().append(data, f.data).isEquals();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getFromJavaConverterInstance() {
        return Float64.getFromJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getToJavaConverterInstance() {
        return Float64.getToJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getTypeSupportInstance() {
        return Float64.getTypeSupport();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getDestructorInstance() {
        return Float64.getDestructor();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final java.lang.String toString() {
        return Double.toString(data);
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();
    
}
