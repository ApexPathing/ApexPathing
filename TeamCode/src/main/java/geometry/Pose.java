package geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * A class representing a 2D position, which consists of a position Vector and a heading Angle.
 * You should consider using a {@link GeometryFactory} to create Pose objects instead of using
 * the constructors in this class directly.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Pose {
    private final Vector position;
    private final Angle heading;

    // region Common poses

    /** Common poses on an FTC field defined in the Apex coordinate system. */
    public enum Common { // Common poses are defined in inches
        CENTER(0, 0),
        TOP_LEFT(-70.5, 70.5),
        TOP_RIGHT(70.5, 70.5),
        BOTTOM_LEFT(-70.5, -70.5),
        BOTTOM_RIGHT(70.5, -70.5);

        private final Vector position;

        Common(double x, double y) { this.position = Vector.of(x, y, DistUnit.IN); }

        /** @return this position as a Vector. */
        public Vector getPosition() { return position; }

        /** @return a Pose with this position and a heading of 0. */
        public Pose get() { return new Pose(position, Angle.fromRad(0)); }

        /** @return a Pose with this position and the specified Angle. */
        public Pose withHeading(Angle heading) { return new Pose(position, heading); }

        /** @return a Pose with this position and the specified heading in the given unit. */
        public Pose withHeading(double heading, AngleUnit angleUnit) {
            return withHeading(Angle.of(heading, angleUnit));
        }
    }

    // endregion
    // region Constructors and factory methods

    /** Creates a Pose from a position Vector and a heading Angle. */
    public Pose(Vector position, Angle heading) {
        this.position = position;
        this.heading = heading;
    }

    /** @return a Pose with X, Y, and heading equal to zero */
    public static Pose zero() { return new Pose(Vector.zero(), Angle.fromRad(0)); }

    // endregion
    // region Getters

    /** @return the position of this Pose as a Vector */
    public Vector getVec() {return position;}

    /** @return the x component of the position as a {@link Dist} */
    public Dist getX() { return position.getX(); }

    /** @return the x component of the position in the specified distance unit */
    public double getX(DistUnit unit) { return position.getX(unit); }

    /** @return the y component of the position as a {@link Dist} */
    public Dist getY() { return position.getY(); }

    /** @return the y component of the position in the specified distance unit */
    public double getY(DistUnit unit) { return position.getY(unit); }

    /** @return the heading of this Pose as an {@link Angle} */
    public Angle getHeading() { return heading; }

    /** @return the heading of this Pose in the specified angle unit */
    public double getHeading(AngleUnit unit) { return heading.get(unit); }

    // endregion
    // region Arithmetic operations

    /** @return a new Pose that is the sum of this Pose and another Pose. */
    public Pose plus(Pose other) {
        return new Pose(this.position.plus(other.position), this.heading.plus(other.heading));
    }

    /** @return a new Pose that is the difference between this Pose and another Pose. */
    public Pose minus(Pose other) {
        return new Pose(this.position.minus(other.position), this.heading.minus(other.heading));
    }

    /** @return a new Pose that is this Pose multiplied by a scalar. */
    public Pose times(double scalar) {
        return new Pose(this.position.times(scalar), this.heading.times(scalar));
    }

    /** @return a new Pose that is this Pose divided by a scalar. */
    public Pose div(double scalar) {
        return new Pose(this.position.div(scalar), this.heading.div(scalar));
    }

    // endregion
    // region Other operations and methods

    /** @return the straight line distance from this vector to another vector */
    public Dist distanceTo(Pose other) { return this.position.distanceTo(other.position); }

    /** @return a new Pose that is the mirror of this Pose across the X-axis. */
    public Pose mirrorX() { return new Pose(this.position.mirrorX(), this.heading.mirrorX()); }

    /** @return a new Pose that is the mirror of this Pose across the Y-axis. */
    public Pose mirrorY() { return new Pose(this.position.mirrorY(), this.heading.mirrorY()); }

    /** @return a copy of this Pose. */
    public Pose copy() { return new Pose(this.position, this.heading); }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() { // ex. (12.000 in, 5.000 in, 3.141 rad)
        return String.format("(%s, %s)", position.toString(), heading.toString());
    }
    // endregion
}