package geometry;

/**
 * Represents a 1D cubic spline for interpolation.
 *
 * <p>
 * This class constructs a cubic spline based on given x and y data points. It provides methods to
 * evaluate the spline and its first and second derivatives at any point within the range of the
 * input data.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class CubicSpline1D {
    private final double[] x;
    private final double[] a, b, c, d;

    public CubicSpline1D(double[] x, double[] y) {
        if (x.length != y.length || x.length < 2) {
            throw new IllegalArgumentException("Spline requires at least 2 points.");
        }

        int n = x.length - 1;
        this.x = x.clone();
        a = y.clone();
        b = new double[n];
        c = new double[n + 1];
        d = new double[n];

        double[] h = new double[n];
        for (int i = 0; i < n; i++) {
            h[i] = x[i + 1] - x[i];
            if (h[i] <= 0)
                throw new IllegalArgumentException("x values must be strictly increasing.");
        }

        double[] alpha = new double[n + 1];

        // Clamped boundary conditions: f'(start) = 0 and f'(end) = 0
        alpha[0] = 3.0 * (a[1] - a[0]) / h[0];
        for (int i = 1; i < n; i++) {
            alpha[i] = 3.0 * (a[i + 1] - a[i]) / h[i] - 3.0 * (a[i] - a[i - 1]) / h[i - 1];
        }
        alpha[n] = -3.0 * (a[n] - a[n - 1]) / h[n - 1];

        double[] l = new double[n + 1];
        double[] mu = new double[n + 1];
        double[] z = new double[n + 1];

        // Forward Elimination
        l[0] = 2.0 * h[0];
        mu[0] = 0.5;
        z[0] = alpha[0] / l[0];

        for (int i = 1; i < n; i++) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 2.0 * h[n - 1] - h[n - 1] * mu[n - 1];
        z[n] = (alpha[n] - h[n - 1] * z[n - 1]) / l[n];
        c[n] = z[n];

        // Back Substitution
        for (int j = n - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }

    private int getSegment(double query) {
        if (query <= x[0]) return 0;
        if (query >= x[x.length - 1]) return x.length - 2;
        for (int i = 0; i < x.length - 1; i++) {
            if (query >= x[i] && query <= x[i + 1]) return i;
        }
        return 0;
    }

    public double evaluate(double query) {
        int i = getSegment(query);
        double dx = query - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

    public double getFirstDerivative(double query) {
        int i = getSegment(query);
        double dx = query - x[i];
        return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
    }

    public double getSecondDerivative(double query) {
        int i = getSegment(query);
        double dx = query - x[i];
        return 2.0 * c[i] + 6.0 * d[i] * dx;
    }
}