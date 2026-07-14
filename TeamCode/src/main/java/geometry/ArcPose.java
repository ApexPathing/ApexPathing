package geometry;

/**
 * Represents a specialized waypoint in a path that should be smoothed into an arc.
 *
 * <p>
 * When passed into a path builder, an {@code ArcPose} acts as a marker indicating that
 * the sharp corner at this location should be dynamically split and rounded.
 * The {@code radius} defines how far from the corner the smoothing begins and ends.
 * </p>
 *
 * @author DrPixelCat
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class ArcPose extends Pose {
    Dist radius;

    /** Creates an ArcPose from a position Vector and a radius. */
    public ArcPose(Vector position, Dist radius) {
        super(position, Angle.fromRad(0));
        this.radius = radius;
    }

    /** @return the radius of this ArcPose. */
    public Dist getRadius() { return radius; }
}