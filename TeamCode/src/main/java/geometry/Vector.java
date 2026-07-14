package geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * A class representing a 2D vector or point using {@link Dist} objects.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public final class Vector {
    private final Dist x;
    private final Dist y;

    // region Constructors and factory methods

    /** Creates a Vector from two Dist objects. */
    public Vector(Dist x, Dist y) {
        this.x = x;
        this.y = y;
    }

    /** Create a Vector with the given (x, y) in the specified unit */
    public static Vector of(double x, double y, DistUnit unit) {
        return new Vector(Dist.of(x, unit), Dist.of(y, unit));
    }

    /** Create a Vector with the given (r, theta) **in polar coordinates** in the specified unit */
    public static Vector fromPolar(Dist r, Angle theta) {
        double radians = theta.getRad();
        return new Vector(
                Dist.fromIn(r.getIn() * Math.cos(radians)),
                Dist.fromIn(r.getIn() * Math.sin(radians))
        );
    }

    /** Create a Vector with X and Y equal to zero */
    public static Vector zero() { return new Vector(Dist.fromIn(0), Dist.fromIn(0)); }

    // endregion
    // region Getters

    /** @return the x {@link Dist} component of the vector */
    public Dist getX() { return x; }

    /** @return the y {@link Dist} component of the vector */
    public Dist getY() { return y; }

    /** @return the x component of the vector in the specified distance unit */
    public double getX(DistUnit unit) { return x.get(unit); }

    /** @return the y component of the vector in the specified distance unit */
    public double getY(DistUnit unit) { return y.get(unit); }

    /** @return the magnitude of this Vector from the origin */
    public Dist getMag() { return x.hypot(y); }

    /** @return the magnitude squared of this Vector from the origin */
    public Dist getMagSq() {
        return Dist.fromIn(x.getIn() * x.getIn() + y.getIn() * y.getIn());
    }

    /** @return the theta of this vector relative to the positive X-axis. */
    public Angle getTheta() { return Angle.fromRad(Math.atan2(y.getIn(), x.getIn())); }

    /** @return true if both components of this Vector are finite numbers (not NaN or infinite). */
    public boolean isFinite() { return Double.isFinite(x.getIn()) && Double.isFinite(y.getIn()); }

    // endregion
    // region Arithmetic operations

    /** @return a Vector that is the sum of this Vector and another Vector */
    public Vector plus(Vector other) {
        return new Vector(this.x.plus(other.x), this.y.plus(other.y));
    }

    /** @return a Vector that is the difference between this Vector and another Vector */
    public Vector minus(Vector other) {
        return new Vector(this.x.minus(other.x), this.y.minus(other.y));
    }

    /** @return a Vector that is this Vector multiplied by a scalar */
    public Vector times(double scalar) {
        return new Vector(this.x.times(scalar), this.y.times(scalar));
    }

    /** @return a Vector that is this Vector divided by a scalar */
    public Vector div(double scalar) { return new Vector(this.x.div(scalar), this.y.div(scalar)); }

    /** @return a Vector that is the absolute value of this Vector */
    public Vector abs() { return new Vector(this.x.abs(), this.y.abs()); }

    /** @return the scalar dot product of this Vector and another Vector */
    public Dist dot(Vector other) { return this.x.times(other.x).plus(this.y.times(other.y)); }

    /** @return the scalar cross product of this Vector and another Vector */
    public Dist cross(Vector other) { return this.x.times(other.y).minus(this.y.times(other.x)); }

    // endregion
    // region Other operations and methods

    /** @return the straight line distance from this vector to another vector */
    public Dist distanceTo(Vector other) { return this.minus(other).getMag(); }

    /** @return a Vector that is this Vector rotated counterclockwise about the origin by theta */
    public Vector rotate(Angle theta) {
        double radians = theta.getRad();
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double xInches = this.x.getIn();
        double yInches = this.y.getIn();

        return new Vector(
                Dist.fromIn(xInches * cos - yInches * sin),
                Dist.fromIn(xInches * sin + yInches * cos)
        );
    }

    /** @return a Vector that is this Vector normalized (scaled to have a magnitude of 1) */
    public Vector normalize() {
        double magIn = this.getMag().getIn();
        if (magIn < 1e-9) {
            return Vector.zero();
        }
        return this.div(magIn);
    }

    /** @return a Vector that is the mirror of this Vector across the X-axis */
    public Vector mirrorX() { return new Vector(this.x, this.y.mirror()); }

    /** @return a Vector that is the mirror of this Vector across the Y-axis */
    public Vector mirrorY() { return new Vector(this.x.mirror(), this.y); }

    /** @return a Vector that is the reflection of this Vector across another Vector */
    public Vector reflect(Vector across) {
        return across.plus(this.minus(across).times(-1.0));
    }

    /** @return a copy of this Vector */
    public Vector copy() { return new Vector(this.x.copy(), this.y.copy()); }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() { return String.format("%s, %s", x.toString(), y.toString()); }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Vector other = (Vector) obj;
        return this.x.equals(other.x) && this.y.equals(other.y);
    }

    // endregion
}
