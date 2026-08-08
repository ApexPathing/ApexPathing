package localizers;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import localizers.util.AdaptiveKalmanFilter;
import localizers.util.DataFilter;
import localizers.util.FilterState;
import localizers.util.LowPassFilter;

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
    private final boolean USE_KALMAN = true; // set this to false if kalman filter has issues to revert to low pass filter

    // A single set of filters. When fed velocity, they output [Velocity, Acceleration].
    private DataFilter xFilter = USE_KALMAN ? new AdaptiveKalmanFilter() : new LowPassFilter();
    private DataFilter yFilter = USE_KALMAN ? new AdaptiveKalmanFilter() : new LowPassFilter();
    private DataFilter headingFilter = USE_KALMAN ? new AdaptiveKalmanFilter() : new LowPassFilter();

    protected enum UpdateType { VELOCITY, ACCELERATION, BOTH }

    protected Pose pose = Pose.zero();
    protected Pose velocity = Pose.zero();
    protected Pose rawVelocity = Pose.zero();
    protected Pose acceleration = Pose.zero();
    protected Pose rawAcceleration = Pose.zero();

    /** Only used for calculating velocity on some localizers */
    private Pose prevPose = Pose.zero();

    /** Only used for calculating acceleration on some localizers */
    private Pose prevRawVelocity = Pose.zero();
    private long prevTimeNs = -1;

    private int lastSize = 0;
    private int FILTER_WINDOW_SIZE = 7; //TODO: Verify this number and make it a constant or delete it
    private boolean isTuning = false;

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

    /**
     * NOTE: ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING
     *  @return the current raw velocity estimate of the robot from the localizer
     */
    public Pose getRawVel() { return rawVelocity; }

    /** @return the current acceleration estimate of the robot from the localizer */
    public Pose getAccel() { return acceleration; }

    /**
     * NOTE: ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING
     *  @return the current raw acceleration estimate of the robot from the localizer
     */
    public Pose getRawAccel() { return rawAcceleration; }

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

        rawVelocity = pose.minus(prevPose).div(dt);
        rawAcceleration = rawVelocity.minus(prevRawVelocity).div(dt);

        // Update sample size if using LowPass
        if (FILTER_WINDOW_SIZE != lastSize && !USE_KALMAN) {
            ((LowPassFilter) xFilter).setSampleSize(FILTER_WINDOW_SIZE);
            ((LowPassFilter) yFilter).setSampleSize(FILTER_WINDOW_SIZE);
            ((LowPassFilter) headingFilter).setSampleSize(FILTER_WINDOW_SIZE);
            lastSize = FILTER_WINDOW_SIZE;
        }

        // Filters MUST be updated every loop to maintain mathematical state continuity,
        // regardless of the UpdateType requested.
        FilterState xState = xFilter.update(rawVelocity.getX().getIn());
        FilterState yState = yFilter.update(rawVelocity.getY().getIn());
        FilterState headingState = headingFilter.update(rawVelocity.getHeading().getRad());

        if (updateType == UpdateType.BOTH || updateType == UpdateType.VELOCITY) {
            velocity = new Pose(
                    new Vector(Dist.fromIn(xState.value()), Dist.fromIn(yState.value())),
                    Angle.fromRad(headingState.value())
            );
        }

        if (isTuning && USE_KALMAN) {
            boolean isMoving = Math.abs(velocity.getX().getIn()) > 1;
            ((AdaptiveKalmanFilter) xFilter).setAutoTuning(isMoving);
            ((AdaptiveKalmanFilter) yFilter).setAutoTuning(isMoving);
            ((AdaptiveKalmanFilter) headingFilter).setAutoTuning(isMoving);
        }

        if (updateType == UpdateType.BOTH || updateType == UpdateType.ACCELERATION) {
            acceleration = new Pose(
                    new Vector(Dist.fromIn(xState.rate()), Dist.fromIn(yState.rate())),
                    Angle.fromRad(headingState.rate())
            );
        }

        prevPose = pose;
        prevRawVelocity = rawVelocity;
        prevTimeNs = currentTimeNs;
    }

    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
        if (!isTuning && USE_KALMAN) {
            ((AdaptiveKalmanFilter) xFilter).setAutoTuning(false);
            ((AdaptiveKalmanFilter) yFilter).setAutoTuning(false);
            ((AdaptiveKalmanFilter) headingFilter).setAutoTuning(false);
        }
    }

    // TUNING INJECTION/EXTRACTION

    public AdaptiveKalmanFilter.KalmanTuning getXKalmanTuning() {
        return USE_KALMAN ? ((AdaptiveKalmanFilter) xFilter).getTuning() : null;
    }

    public void setXKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (USE_KALMAN && tuning != null) {
            ((AdaptiveKalmanFilter) xFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getYKalmanTuning() {
        return USE_KALMAN ? ((AdaptiveKalmanFilter) yFilter).getTuning() : null;
    }

    public void setYKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (USE_KALMAN && tuning != null) {
            ((AdaptiveKalmanFilter) yFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getHeadingKalmanTuning() {
        return USE_KALMAN ? ((AdaptiveKalmanFilter) headingFilter).getTuning() : null;
    }

    public void setHeadingKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (USE_KALMAN && tuning != null) {
            ((AdaptiveKalmanFilter) headingFilter).setTuning(tuning);
        }
    }

    // TODO: Delete this temporary method once a value is settled on
    /**
     * FOR INTERNAL USE ONLY, DO NOT USE
     * @param windowSize the size of the filter window
     */
    public void setFilterWindow(int windowSize) {
        this.FILTER_WINDOW_SIZE = windowSize;
    }
}