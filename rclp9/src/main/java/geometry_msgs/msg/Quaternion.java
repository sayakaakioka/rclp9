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
public class Quaternion implements MessageDefinition {
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

    private double x;
    private double y;
    private double z;
    private double w;

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode(){
        return new HashCodeBuilder().append(x).append(y).append(z).append(w).toHashCode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(final Object obj){
        if(!(obj instanceof Quaternion)){
            return false;
        }

        if(this == obj){
            return true;
        }

        Quaternion q = (Quaternion)obj;
        return new EqualsBuilder().append(x, q.x).append(y, q.y).append(z, q.z).append(w, q.w).isEquals();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getFromJavaConverterInstance() {
        return Quaternion.getFromJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getToJavaConverterInstance() {
        return Quaternion.getToJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getTypeSupportInstance() {
        return Quaternion.getTypeSupport();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getDestructorInstance() {
        return Quaternion.getDestructor();
    }

    /**
     * Sets the value in Quaternion.
     * @param q the value in Quaternion
     */
    public void quaternion(final Quaternion q){
        this.x(q.x());
        this.y(q.y());
        this.z(q.z());
        this.w(q.w());
    }

    /**
     * Sets the values of (x, y, z, w).
     * @param x x in double
     * @param y y in double
     * @param z z in double
     * @param w w in double
     */
    public void quaternion(final double x, final double y, final double z, final double w){
        this.x(x);
        this.y(y);
        this.z(z);
        this.w(w);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final java.lang.String toString() {
        return "("+ x + ", " + y + ", " + z + ", " + w + ")";
    }

    /**
     * Gets the value of w.
     * @return w in double
     */
    public double w(){
        return w;
    }

    /**
     * Sets the value of w.
     * @param w the value to be set
     */
    public void w(final double w){
        this.w = w;
    }

    /**
     * Gets the value of x.
     * @return x in double
     */
    public double x(){
        return x;
    }

    /**
     * Sets the value of x.
     * @param x the value to be set
     */
    public void x(final double x){
        this.x = x;
    }

    /**
     * Gets the value of y.
     * @return y in double
     */
    public double y(){
        return y;
    }

    /**
     * Sets the value of y.
     * @param y the value to be set
     */
    public void y(final double y){
        this.y = y;
    }

    /**
     * Gets the value of z.
     * @return z in double
     */
    public double z(){
        return z;
    }
    
    /**
     * Sets the value of z.
     * @param z the value to be set
     */
    public void z(final double z){
        this.z = z;
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();
    
}
