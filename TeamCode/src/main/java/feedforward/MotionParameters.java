package feedforward;

/**
 * One row of a feedforward trajectory lookup table.
 *
 * <p>This class stores the path-relative kinematic state as {@code [v, a, omega, alpha]}:
 * tangential velocity, tangential acceleration, angular velocity, and angular acceleration,
 * plus bookkeeping values used by the follower and generator.
 *
 * @author DrPixelCat - 7842 alum
 */
public class MotionParameters {
    /** Distance units per second. */
    private double tangentialVel;
    /** Distance units per second squared. */
    private double tangentialAccel;
    /** Radians per second. */
    private double angularVel;
    /** Radians per second squared. */
    private double angularAccel;
    private double distAlongCurve;
    private double motorPower = 0.0;

    /**
     * Creates a blank parameters object.
     *
     * <p>Generators use this when they fill the values in several passes.
     */
    public MotionParameters() {
        this(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Creates a fully initialized trajectory row.
     *
     * @param tangentialVel path-relative linear velocity
     * @param tangentialAccel path-relative linear acceleration
     * @param angularVel robot heading velocity
     * @param angularAccel robot heading acceleration
     * @param distAlongCurve interpolation key for this row
     */
    public MotionParameters(double tangentialVel, double tangentialAccel, double angularVel,
                            double angularAccel, double distAlongCurve) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
        this.distAlongCurve = distAlongCurve;
    }

    /** Sets tangential velocity and returns this object for chaining. */
    public MotionParameters setTangentialVel(double tangentialVel) {
        this.tangentialVel = tangentialVel;
        return this;
    }

    /** Sets tangential acceleration and returns this object for chaining. */
    public MotionParameters setTangentialAccel(double tangentialAccel) {
        this.tangentialAccel = tangentialAccel;
        return this;
    }

    /** Sets angular velocity and returns this object for chaining. */
    public MotionParameters setAngularVel(double angularVel) {
        this.angularVel = angularVel;
        return this;
    }

    /** Sets angular acceleration and returns this object for chaining. */
    public MotionParameters setAngularAccel(double angularAccel) {
        this.angularAccel = angularAccel;
        return this;
    }

    /** @return normalized motor utilization estimate for this row */
    public double getMotorPower() { return motorPower; }

    /** Stores the normalized motor utilization estimate for this row. */
    public void setMotorPower(double motorPower) { this.motorPower = motorPower; }

    /**
     * Sets the interpolation key for this row ( whatever the path/follower uses as progression).
     */
    public void setDistAlongCurve(double distAlongCurve) { this.distAlongCurve = distAlongCurve; }

    /** @return path-relative linear velocity */
    public double getTangentialVel() { return tangentialVel; }

    /** @return path-relative linear acceleration */
    public double getTangentialAccel() { return tangentialAccel; }

    /** @return robot heading velocity */
    public double getAngularVel() { return angularVel; }

    /** @return robot heading acceleration */
    public double getAngularAccel() { return angularAccel; }

    /** @return interpolation key stored for this row */
    public double getDistAlongCurve() { return distAlongCurve; }

    /** @return interpolation key used by {@link FFLut} */
    public double getProgression() { return distAlongCurve; }
}
