package localizers;

import geometry.Pose;

/**
 * Base class for all localizers.
 *
 * <p>
 * This class provides common properties and methods for localizers, such as storing the current
 * factory, velocity, and acceleration estimates. Specific localizer types (like odometry, IMU-based,
 * etc.) should extend this class and implement the update() method that updates these estimates
 * based on sensor data.
 * </p>
 *
 * @param <T> the type of localizer configuration this drivetrain uses, which must extend
 *        {@link BaseLocalizerConstants}
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseLocalizer<T extends BaseLocalizerConstants<T>> {
    protected T config;

    protected enum UpdateType { VELOCITY, ACCELERATION, BOTH }

    protected Pose pose = Pose.zero();
    protected Pose velocity = Pose.zero();
    protected Pose acceleration = Pose.zero();

    // Only used for calculating velocity on some localizers
    private Pose prevPose = Pose.zero();

    // Only used for calculating acceleration on some localizers
    private Pose prevVelocity = Pose.zero();
    private long prevTimeNs = -1;

    /**
     * Your localizer class constructor should call this super constructor to store the
     * configuration.
     *
     * @param config your localizer configuration object that is a child of
     *               {@link BaseLocalizerConstants}
     */
    public BaseLocalizer(T config) { this.config = config; }

    /** @return the current factory estimate of the robot from the localizer */
    public Pose getPose() { return pose; }

    /** @return the current velocity estimate of the robot from the localizer */
    public Pose getVel() { return velocity; }

    /** @return the current acceleration estimate of the robot from the localizer */
    public Pose getAccel() { return acceleration; }

    /**
     * Update the localizer's factory, velocity, and acceleration estimates. This method should be
     * called regularly in a loop. If your localizer doesn't give velocity and/or acceleration, you
     * can use the calculate() method to update one or both using math
     */
    public abstract void update();

    /**
     * Set the localizer's current factory estimate with the given {@link Pose}
     * Note: Don't worry about updating this classes factory field, it will be updated in the next
     * update() call.
     */
    public abstract void setPose(Pose newPose);

    /**
     * Calculates the current velocity and/or acceleration for localizers that don't natively
     * support it
     **/
    protected void calculate(UpdateType updateType) {
        long currentTimeNs = System.nanoTime();

        if (prevTimeNs == -1) {
            prevTimeNs = currentTimeNs;
            prevPose = pose;
            return;
        }

        double dt = (currentTimeNs - prevTimeNs) / 1_000_000_000.0;
        if (dt <= 1e-6) { return; }

        if (updateType == UpdateType.BOTH || updateType == UpdateType.VELOCITY) {
            velocity = pose.minus(prevPose).div(dt); // v = dp / dt
            prevPose = pose;
        }
        if (updateType == UpdateType.BOTH || updateType == UpdateType.ACCELERATION) {
            acceleration = velocity.minus(prevVelocity).div(dt); // a = dv / dt
            prevVelocity = velocity;
        }

        prevTimeNs = currentTimeNs;
    }
}