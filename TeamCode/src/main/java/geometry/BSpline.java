package geometry;

/**
 * Represents a Uniform Cubic B-Spline.
 *
 * <p>
 * B-Splines guarantee C2 continuity (smooth position, velocity, and acceleration)
 * across the entire path. Because they are evaluated using a sliding 4-point window,
 * calculating a point on the curve runs in O(1) constant time, regardless of how
 * many control points are in the path.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22895 Paraducks
 */
public class BSpline implements ParametricSegment {
    private final int numSegments;

    // Cached polynomial coefficients for each segment
    // cx[segmentIndex] returns the [c3, c2, c1, c0] array for that segment
    private final double[][] cx;
    private final double[][] cy;

    private static final Matrix BLEND_MATRIX = new Matrix(new double[][]{
            {-1.0 / 6.0, 3.0 / 6.0, -3.0 / 6.0, 1.0 / 6.0},
            {3.0 / 6.0, -6.0 / 6.0, 3.0 / 6.0, 0.0},
            {-3.0 / 6.0, 0.0, 3.0 / 6.0, 0.0},
            {1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0, 0.0}
    });

    /**
     * Constructs a continuous B-Spline from an array of waypoints.
     * Automatically generates "ghost points" at the start and end to guarantee
     * the curve properly anchors to the first and last input points.
     *
     * @param inputPoints An array of Vector waypoints the spline is built around.
     * @throws IllegalArgumentException if there are 1 or fewer points provided.
     */
    public BSpline(Vector[] inputPoints) throws IllegalArgumentException {
        if (inputPoints.length < 2) {
            throw new IllegalArgumentException("You can't make a B-Spline curve with < 2 points!");
        }

        // Create ghost points
        Vector[] paddedPoints = new Vector[inputPoints.length + 2];
        paddedPoints[0] = inputPoints[1].reflect(inputPoints[0]);
        paddedPoints[paddedPoints.length - 1] =
                inputPoints[inputPoints.length - 2].reflect(inputPoints[inputPoints.length - 1]);
        System.arraycopy(inputPoints, 0, paddedPoints, 1, inputPoints.length);

        // Precompute and cache coefficients for all segments
        this.numSegments = paddedPoints.length - 3;
        this.cx = new double[numSegments][4];
        this.cy = new double[numSegments][4];

        for (int i = 0; i < numSegments; i++) {
            Vector p0 = paddedPoints[i];
            Vector p1 = paddedPoints[i + 1];
            Vector p2 = paddedPoints[i + 2];
            Vector p3 = paddedPoints[i + 3];

            double[] xWindow = {p0.getX().getIn(), p1.getX().getIn(), p2.getX().getIn(),
                    p3.getX().getIn()};
            double[] yWindow = {p0.getY().getIn(), p1.getY().getIn(), p2.getY().getIn(),
                    p3.getY().getIn()};

            this.cx[i] = BLEND_MATRIX.multiply(xWindow);
            this.cy[i] = BLEND_MATRIX.multiply(yWindow);
        }
    }

    /**
     * Calculates the physical (x, y) position on the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the coordinate location.
     */
    @Override
    public Vector getPosition(double t) {
        if (t >= 1.0) t = 0.999999;
        if (t < 0.0) t = 0.0;

        double continuousIndex = t * numSegments;
        int segment = (int) continuousIndex;
        double localT = continuousIndex - segment;

        // Grab precomputed coefficients
        double[] cX = cx[segment];
        double[] cY = cy[segment];

        // Position: c3*t^3 + c2*t^2 + c1*t + c0
        double x = ((cX[0] * localT + cX[1]) * localT + cX[2]) * localT + cX[3];
        double y = ((cY[0] * localT + cY[1]) * localT + cY[2]) * localT + cY[3];

        return Vector.of(x, y, DistUnit.IN);
    }

    /**
     * Calculates the first derivative (velocity vector) of the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the parametric velocity.
     */
    @Override
    public Vector getFirstDerivative(double t) {
        if (t >= 1.0) t = 0.999999;
        if (t < 0.0) t = 0.0;

        double continuousIndex = t * numSegments;
        int segment = (int) continuousIndex;
        double localT = continuousIndex - segment;

        double[] cX = cx[segment];
        double[] cY = cy[segment];

        // Velocity: 3*c3*t^2 + 2*c2*t + c1
        double dx = (3.0 * cX[0] * localT + 2.0 * cX[1]) * localT + cX[2];
        double dy = (3.0 * cY[0] * localT + 2.0 * cY[1]) * localT + cY[2];

        // Chain rule scaling
        return Vector.of(dx, dy, DistUnit.IN).times(numSegments);
    }

    /**
     * Calculates the second derivative (acceleration vector) of the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the parametric acceleration.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        if (t >= 1.0) t = 0.999999;
        if (t < 0.0) t = 0.0;

        double continuousIndex = t * numSegments;
        int segment = (int) continuousIndex;
        double localT = continuousIndex - segment;

        double[] cX = cx[segment];
        double[] cY = cy[segment];

        // Acceleration: 6*c3*t + 2*c2
        double ddx = 6.0 * cX[0] * localT + 2.0 * cX[1];
        double ddy = 6.0 * cY[0] * localT + 2.0 * cY[1];

        // Chain rule scaling
        return Vector.of(ddx, ddy, DistUnit.IN).times((double) numSegments * numSegments);
    }
}
