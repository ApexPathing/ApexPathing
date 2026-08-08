package geometry;

/**
 * Represents a precalculated, discrete point along a parametric path segment.
 *
 * <p>This class is used to populate a Look-Up Table (LUT), allowing the robot to quickly map a
 * parametric 't' value to physical coordinates and estimate the remaining arc-length distance to
 * the end of the curve in O(1) time.
 *
 * @author DrPixelCat - 7842 alum
 */
public class PathPoint {
    private final double t;
    private final double distanceToEnd;
    private final Vector location;
    private final Vector firstDerivative;
    private final double curvature;
    private final double curvatureDerivative;

    /**
     * Constructs a precalculated path point.
     *
     * @param t The parametric value [0.0, 1.0] representing this point's location on the curve.
     * @param distanceToEnd The calculated arc length from this point to the end of the segment.
     * @param location The physical 2D coordinate of the curve at this 't' value.
     */
    public PathPoint(double t, double distanceToEnd, Vector location, Vector firstDerivative,
                     double curvature, double curvatureDerivative) {
        this.t = t;
        this.distanceToEnd = distanceToEnd;
        this.location = location;
        this.firstDerivative = firstDerivative;
        this.curvature = curvature;
        this.curvatureDerivative = curvatureDerivative;
    }

    /** @return The location of this point as a Vector. */
    public Vector getLocation() { return location; }

    /** @return The 't' value, typically between [0.0, 1.0]. */
    public double getT() { return t; }

    /** @return The first derivative (tangent) of the point */
    public Vector getFirstDerivative() { return firstDerivative; }

    /** @return The distance of this point to the end of the segment in inches. */
    public double getDistanceToEndIn() { return distanceToEnd; }

    /** @return The curvature at parameter t of the curve. */
    public double getSignedCurvature() { return curvature; }

    /**  @return The curvature's derivative at parameter t of the curve. */
    public double getCurvatureDerivative() { return curvatureDerivative; }
}