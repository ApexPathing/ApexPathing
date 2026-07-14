package feedforward;

/**
 * One row of a feedforward trajectory lookup table.
 *
 * <p>
 * The paper describes the path-relative kinematic state as {@code [v, a, omega, alpha]}:
 * tangential velocity, tangential acceleration, angular velocity, and angular acceleration.
 * This class stores that state, plus bookkeeping values used by the follower and generator.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class MotionParameters {
    /** Path-relative linear speed, in distance units per second. */
    private double tangentialVel;
    /** Path-relative linear acceleration, in distance units per second squared. */
    private double tangentialAccel;
    /** Robot heading velocity, in radians per second. */
    private double angularVel;
    /** Robot heading acceleration, in radians per second squared. */
    private double angularAccel;
    /** Distance/progression key used to interpolate this row from the LUT. */
    private double distAlongCurve = 0.0;
    /** Normalized drivetrain utilization estimated by the generator. */
    private double motorPower = 0.0;

    /**
     * Creates a blank parameters object.
     * <p>
     * Generators use this when they fill the values in several passes.
     */
    public MotionParameters() {
        this.tangentialVel = 0.0;
        this.tangentialAccel = 0.0;
        this.angularVel = 0.0;
        this.angularAccel = 0.0;
        this.distAlongCurve = 0.0;
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

    /**
     * Creates a kinematic row without an interpolation key.
     *
     * @param tangentialVel path-relative linear velocity
     * @param tangentialAccel path-relative linear acceleration
     * @param angularVel robot heading velocity
     * @param angularAccel robot heading acceleration
     */
    public MotionParameters(double tangentialVel, double tangentialAccel, double angularVel,
                            double angularAccel) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
    }

    /**
     * Sets tangential velocity and returns this object for pass-style chaining.
     */
    public MotionParameters setTangentialVel(double tangentialVel) {
        this.tangentialVel = tangentialVel;
        return this;
    }

    /**
     * Sets tangential acceleration and returns this object for pass-style chaining.
     */
    public MotionParameters setTangentialAccel(double tangentialAccel) {
        this.tangentialAccel = tangentialAccel;
        return this;
    }

    /**
     * Sets angular velocity and returns this object for pass-style chaining.
     */
    public MotionParameters setAngularVel(double angularVel) {
        this.angularVel = angularVel;
        return this;
    }

    /**
     * Sets angular acceleration and returns this object for pass-style chaining.
     */
    public MotionParameters setAngularAccel(double angularAccel) {
        this.angularAccel = angularAccel;
        return this;
    }

    /**
     * @return normalized motor utilization estimate for this row
     */
    public double getMotorPower() {return motorPower;}

    /**
     * Stores the normalized motor utilization estimate for this row.
     */
    public void setMotorPower(double motorPower) {this.motorPower = motorPower;}

    /**
     * Sets the interpolation key for this row.
     * <p>
     * In this codebase the key is whatever the path/follower uses as progression.
     */
    public void setDistAlongCurve(double distAlongCurve) {
        this.distAlongCurve = distAlongCurve;
    }

    /** @return path-relative linear velocity */
    public double getTangentialVel() {
        return tangentialVel;
    }

    /** @return path-relative linear acceleration */
    public double getTangentialAccel() {
        return tangentialAccel;
    }

    /** @return robot heading velocity */
    public double getAngularVel() {
        return angularVel;
    }

    /** @return robot heading acceleration */
    public double getAngularAccel() {
        return angularAccel;
    }

    /** @return interpolation key stored for this row */
    public double getDistAlongCurve() {
        return distAlongCurve;
    }

    /**
     * @return interpolation key used by {@link FeedforwardLut}
     */
    public double getProgression() {
        return distAlongCurve;
    }
}
