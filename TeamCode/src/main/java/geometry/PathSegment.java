package geometry;

/**
 * A wrapper class that binds a mathematical parametric curve to physical properties.
 *
 * <p>
 * This class handles the generation of a Look-Up Table (LUT) to precalculate
 * arc-length distances, enabling blisteringly fast O(1) distance lookups and
 * highly efficient closest-point projection using Newton-Raphson refinement.
 * Internally, all units are inches and radians.
 * <p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class PathSegment {
    private static final double POINTS_PER_INCH = 0.5;

    private final ParametricSegment segment;
    private final double length;
    private final PathPoint[] LUTpoints;

    /**
     * Constructs a PathSegment and automatically generates its Look-Up Table (LUT).
     *
     * @param segment The underlying parametric geometry (e.g., Line, BSpline).
     */
    public PathSegment(ParametricSegment segment) {
        this.segment = segment;
        double length = calculateCoarseLength();

        int calculatedPoints = (int) (length * POINTS_PER_INCH);
        int numPoints = Math.max(2, calculatedPoints);

        this.LUTpoints = new PathPoint[numPoints];

        double distFromEnd = 0.0;
        Vector lastPoint = null;

        for (int i = numPoints - 1; i >= 0; i--) {
            double t = (double) i / (numPoints - 1);
            Vector location = segment.getPosition(t);

            if (lastPoint != null) {
                distFromEnd += lastPoint.minus(location).getMag().getIn();
            }
            lastPoint = location;
            LUTpoints[i] = new PathPoint(t, distFromEnd, location,
                    getFirstDerivative(t), getSignedCurvature(t), getCurvatureDerivative(t));
        }

        this.length = distFromEnd;
    }

    /**
     * Finds the closest parametric 't' value on the curve to the robot's physical location.
     * Uses a highly efficient two-step process: an O(N) coarse search through the LUT,
     * followed by a continuous Newton-Raphson root-finding refinement.
     *
     * @param location The current physical coordinate of the robot.
     * @return The parametric value 't' [0.0, 1.0] of the closest point on the curve.
     */
    public double getBestT(Vector location) {
        double bestT = 0;
        double minDistSq = Double.MAX_VALUE;

        // Coarse search via LUT
        for (PathPoint point : LUTpoints) {
            double distSq = point.getLocation().minus(location).getMagSq().getIn();

            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestT = point.getT();
            }
        }

        // Newton-Raphson refinement
        for (int i = 0; i < 5; i++) {
            Vector b = segment.getPosition(bestT);
            Vector d1 = segment.getFirstDerivative(bestT);

            Vector diff = b.minus(location);
            double numerator = diff.dot(d1).getIn();

            // If the distance is orthogonal to the tangent we already found the closest point
            if (Math.abs(numerator) < 1e-6 && bestT > 0.0 && bestT < 1.0) {
                break;
            }

            Vector d2 = segment.getSecondDerivative(bestT);
            double denominator = d1.dot(d1).plus(diff.dot(d2)).getIn();

            // Abort on singularity to prevent backward pushing
            if (denominator <= 0.0) {
                break;
            }

            double previousT = bestT;
            bestT = bestT - (numerator / denominator);
            bestT = Math.max(0.0, Math.min(1.0, bestT));

            if (Math.abs(bestT - previousT) < 1e-6) {
                break;
            }
        }

        // Update state at refined t
        return bestT;
    }

    /**
     * Retrieves the physical coordinate of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The 2D position Vector.
     */
    public Vector getPosition(double t) { return segment.getPosition(t); }

    /**
     * Retrieves the first derivative (velocity) of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The velocity Vector.
     */
    public Vector getFirstDerivative(double t) { return segment.getFirstDerivative(t); }

    /**
     * Retrieves the second derivative (acceleration) of the curve at a given 't'.
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The acceleration Vector.
     */
    public Vector getSecondDerivative(double t) { return segment.getSecondDerivative(t); }

    /**
     * Calculates the remaining physical distance to the end of the segment
     * using a blisteringly fast O(1) LUT index calculation.
     *
     * @param closestPointOnCurve The calculated physical position on the curve closest to the
     *                            robot.
     * @param t The parametric 't' value that yielded closestPointOnCurve.
     * @return The remaining distance in inches.
     */
    public double getDistanceToEndIn(Vector closestPointOnCurve, double t) {
        if (t >= 1.0) return 0.0;

        if (t <= 0.0) {
            double mag = closestPointOnCurve.minus(LUTpoints[0].getLocation()).getMag().getIn();
            return mag + LUTpoints[0].getDistanceToEndIn();
        }

        int lastIndex = LUTpoints.length - 1;
        int nextIndex = (int) Math.ceil(t * lastIndex);
        nextIndex = Math.max(0, Math.min(nextIndex, lastIndex));
        PathPoint nextPoint = LUTpoints[nextIndex];

        double mag = closestPointOnCurve.minus(nextPoint.getLocation()).getMag().getIn();
        return mag + nextPoint.getDistanceToEndIn();
    }

    /**
     * A highly optimized approximation of the segment's length used exclusively
     * to determine how many LUT points to allocate.
     *
     * @return An estimated arc-length in inches.
     */
    private double calculateCoarseLength() {
        final int SAMPLES = 8;
        double roughLength = 0.0;
        Vector prev = segment.getPosition(0.0);
        for (int i = 1; i <= SAMPLES; i++) {
            Vector curr = segment.getPosition((double) i / SAMPLES);
            roughLength += curr.minus(prev).getMag().getIn();
            prev = curr;
        }
        return roughLength;
    }

    /** @return The high-accuracy calculated length of the segment in inches. */
    public double getLengthIn() { return length; }

    /**
     * Calculates the instantaneous radius of curvature of a parametric curve at a specific point.
     *
     * <p>
     * The radius of curvature is geometrically defined as the radius of the circular arc
     * which best approximates the curve at that point. It is computed using the magnitude
     * of the first derivative cubed, divided by the magnitude of the 2D cross product
     * of the first and second derivatives.
     * </p>
     *
     * @param firstDerivative The first derivative vector (velocity/tangent) of the curve.
     * @param secondDerivative The second derivative vector (acceleration) of the curve.
     * @return The instantaneous radius of curvature. Returns Double.POSITIVE_INFINITY if the
     *         curve is perfectly straight.
     */
    public static double calculateRadiusOfCurvature(Vector firstDerivative,
                                                    Vector secondDerivative) {
        double crossProductMag = firstDerivative.cross(secondDerivative).abs().getIn();

        // If the cross product is near zero, the derivatives are parallel, meaning the path is a
        // perfectly straight line with an infinite radius.
        if (crossProductMag < 1e-6) {return Double.POSITIVE_INFINITY;}

        double velocityMag = firstDerivative.getMag().getIn();
        double numerator = Math.pow(velocityMag, 3);

        return numerator / crossProductMag;
    }

    /**
     * Estimates the derivative of curvature with respect to arc length (dK/ds)
     * using a central finite difference method.
     *
     * <p>
     * To prevent floating-point precision loss or computational instability on extremely
     * long or short segments, the delta 't' (dt) is dynamically scaled based on the
     * physical length of the curve to evaluate across a consistent physical distance.
     * </p>
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The estimated rate of change of signed curvature in 1/in^2.
     */
    public double getCurvatureDerivative(double t) {
        final double targetPhysicalDelta_in = 0.1;
        double dt = targetPhysicalDelta_in / Math.max(this.length, 1e-6);

        dt = Math.max(1e-5, Math.min(dt, 0.05));

        double t1 = t - dt;
        double t2 = t + dt;

        // Shift the window to a forward or backward difference if we hit the [0, 1] bounds
        if (t1 < 0.0) {
            t1 = 0.0;
            t2 = 2.0 * dt;
        } else if (t2 > 1.0) {
            t2 = 1.0;
            t1 = 1.0 - 2.0 * dt;
        }

        double k1 = getSignedCurvature(t1);
        double k2 = getSignedCurvature(t2);
        double ds = segment.getPosition(t2).minus(segment.getPosition(t1)).getMag().getIn();

        if (ds < 1e-6) {
            return 0.0;
        }

        return (k2 - k1) / ds;
    }

    /**
     * Calculates the signed curvature at a given parameter 't'.
     *
     * <p>
     * Unlike the radius of curvature, signed curvature retains the direction of the bend
     * (positive vs. negative). This is mathematically required to correctly calculate
     * continuous derivatives across inflection points where the path changes bend direction.
     * </p>
     *
     * @param t The parametric progression [0.0, 1.0].
     * @return The instantaneous signed curvature.
     */
    public double getSignedCurvature(double t) {
        Vector v = segment.getFirstDerivative(t);
        Vector a = segment.getSecondDerivative(t);

        // The 2D cross product retains the sign (left vs right bend)
        double cross = v.cross(a).getIn();
        double vMag = v.getMag().getIn();

        // Safety Check: Prevent division by zero if the robot is momentarily stationary
        if (vMag < 1e-6) {
            return 0.0;
        }

        // k = (x'y'' - y'x'') / ||v||^3
        return cross / Math.pow(vMag, 3);
    }

    public PathPoint[] getPointLUT() {
        return LUTpoints;
    }

    /**
     * Retrieves the 2D principal unit normal vector to the curve at a given 't'.
     *
     * <p>
     * The principal normal points strictly towards the center of curvature (the "inside"
     * of the curve). For maximum efficiency, this avoids trigonometric functions by swapping
     * coordinates, and uses the 2D cross product of velocity and acceleration to determine
     * the bend direction.
     * </p>
     *
     * @param firstDerivative  The velocity vector of the segment at the closest point.
     * @param secondDerivative The acceleration vector of the segment at the closest point.
     * @return The principal unit normal Vector pointing toward the center of curvature. Returns
     * (0, 0) if the cross product is zero.
     */
    public static Vector calculateArcNormal(Vector firstDerivative, Vector secondDerivative) {
        double vx = firstDerivative.getX().getIn();
        double vy = firstDerivative.getY().getIn();
        double cross = firstDerivative.cross(secondDerivative).getIn();

        // If the path is perfectly straight, there is no normal vector
        if (Math.abs(cross) < 1e-6) {return Vector.zero();}

        Vector normal;
        if (cross < 0) {
            // Bending right: swap X and Y, negate the new Y
            normal = Vector.of(vy, -vx, DistUnit.IN);
        } else {
            // Bending left: swap X and Y, negate the new X
            normal = Vector.of(-vy, vx, DistUnit.IN);
        }

        return normal.normalize();
    }
}
