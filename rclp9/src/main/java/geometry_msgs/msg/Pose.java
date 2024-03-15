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
public class Pose implements MessageDefinition {
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

    private Point position = new Point();
    private Quaternion orientation = new Quaternion();

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode(){
        return new HashCodeBuilder().append(position).append(orientation).toHashCode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(final Object obj){
        if(!(obj instanceof Pose)){
            return false;
        }

        if(this == obj){
            return true;
        }

        Pose p = (Pose)obj;
        return new EqualsBuilder().append(position, p.position).append(orientation, p.orientation).isEquals();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getFromJavaConverterInstance() {
        return Pose.getFromJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getToJavaConverterInstance() {
        return Pose.getToJavaConverter();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getTypeSupportInstance() {
        return Pose.getTypeSupport();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final long getDestructorInstance() {
        return Pose.getDestructor();
    }

    /**
     * Sets the values of (position, orientation).
     * @param position position in Point
     * @param orientation orientation in double
     */
    public void pose(final Point position, final Quaternion orientation){
        this.position(position);
        this.orientation(orientation);
    }

    public void pose(final double px, final double py, final double pz, 
        final double ox, final double oy, final double oz, final double ow){
            this.position(px, py, pz);
            this.orientation(ox, oy, oz, ow);
    }

    /**
     * Gets the value of position.
     * @return position in Point
     */
    public Point position(){
        return position;
    }

    /**
     * Sets the value of position.
     * @param p the value to be set
     */
    public void position(final Point p){
        this.position = p;
    }

    /**
     * Sets the values in double into position 
     * @param x x in double
     * @param y y in double
     * @param z z in double
     */
    public void position(final double x, final double y, final double z){
        this.position.point(x, y, z);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public final java.lang.String toString() {
        return "position = " + position + ", orientation = " + orientation;
    }

    /**
     * Gets the value of orientation.
     * @return o in Quaternion
     */
    public Quaternion orientation(){
        return orientation;
    }

    /**
     * Sets the value of orientation.
     * @param o the value to be set
     */
    public void orientation(final Quaternion q){
        this.orientation = q;
    }

    /**
     * Sets the values in double into orientation
     * @param x x in double
     * @param y y in double
     * @param z z in double
     * @param w w in double
     */
    public void orientation(final double x, final double y, final double z, final double w){
        this.orientation.quaternion(x, y, z, w);
    }

    private static native final long getDestructor();

    private static native final long getFromJavaConverter();

    private static native final long getToJavaConverter();

    private static native final long getTypeSupport();
    
}
