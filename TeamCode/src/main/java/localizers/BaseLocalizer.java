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
    private static final AdaptiveKalmanFilter.KalmanTuning DEFAULT_TRANSLATION_KALMAN =
            new AdaptiveKalmanFilter.KalmanTuning(0.25, 625.0);
    private static final AdaptiveKalmanFilter.KalmanTuning DEFAULT_HEADING_KALMAN =
            new AdaptiveKalmanFilter.KalmanTuning(0.01, 25.0);
    public enum VelocityFilterMode {
        ADAPTIVE_KALMAN,
        MOVING_AVERAGE
    }

    protected T config;
    private VelocityFilterMode velocityFilterMode;

    // A single set of filters. When fed velocity, they output [Velocity, Acceleration].
    private DataFilter xFilter;
    private DataFilter yFilter;
    private DataFilter headingFilter;

    protected enum UpdateType { VELOCITY, ACCELERATION, BOTH }

    protected Pose pose = Pose.zero();
    protected Pose velocity = Pose.zero();
    protected Pose rawVelocity = Pose.zero();
    protected Pose acceleration = Pose.zero();
    protected Pose rawAcceleration = Pose.zero();

    /** History shared by pose-derived and native-velocity localizers. */
    private Pose prevPose = Pose.zero();
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
    public BaseLocalizer(T config) {
        this(config, VelocityFilterMode.ADAPTIVE_KALMAN);
    }

    protected BaseLocalizer(T config, VelocityFilterMode velocityFilterMode) {
        this.config = config;
        setVelocityFilterModeInternal(velocityFilterMode);
    }

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

    /** @return the velocity filter's moving-average window size in samples */
    public int getFilterWindowSize() { return FILTER_WINDOW_SIZE; }

    public VelocityFilterMode getVelocityFilterMode() { return velocityFilterMode; }

    /**
     * Selects the canonical public velocity estimator. Changing modes clears only kinematic
     * history; the current pose is preserved.
     */
    public void setVelocityFilterMode(VelocityFilterMode mode) {
        if (mode == null || mode == velocityFilterMode) { return; }
        setVelocityFilterModeInternal(mode);
        resetKinematicEstimate(pose);
    }

    private void setVelocityFilterModeInternal(VelocityFilterMode mode) {
        velocityFilterMode = mode == null ? VelocityFilterMode.ADAPTIVE_KALMAN : mode;
        if (velocityFilterMode == VelocityFilterMode.ADAPTIVE_KALMAN) {
            xFilter = new AdaptiveKalmanFilter();
            yFilter = new AdaptiveKalmanFilter();
            headingFilter = new AdaptiveKalmanFilter();
            ((AdaptiveKalmanFilter) xFilter).setTuning(DEFAULT_TRANSLATION_KALMAN);
            ((AdaptiveKalmanFilter) yFilter).setTuning(DEFAULT_TRANSLATION_KALMAN);
            ((AdaptiveKalmanFilter) headingFilter).setTuning(DEFAULT_HEADING_KALMAN);
        } else {
            xFilter = new LowPassFilter();
            yFilter = new LowPassFilter();
            headingFilter = new LowPassFilter();
            applyMovingAverageWindow();
        }
    }

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

        applyVelocityMeasurement(pose.minus(prevPose).div(dt), updateType, dt);
        prevPose = pose;
        prevTimeNs = currentTimeNs;
    }

    /**
     * Filters a native velocity measurement and derives acceleration from the same filter state.
     * Localizers with hardware-provided velocity should use this instead of assigning
     * {@link #velocity} directly.
     */
    protected void calculate(Pose measuredVelocity) {
        long currentTimeNs = System.nanoTime();
        if (prevTimeNs == -1) {
            rawVelocity = measuredVelocity;
            rawAcceleration = Pose.zero();
            applyFilterState(UpdateType.BOTH);
            prevPose = pose;
            prevRawVelocity = measuredVelocity;
            prevTimeNs = currentTimeNs;
            return;
        }

        double dt = (currentTimeNs - prevTimeNs) / 1_000_000_000.0;
        if (dt <= 1e-6) { return; }

        applyVelocityMeasurement(measuredVelocity, UpdateType.BOTH, dt);
        prevPose = pose;
        prevTimeNs = currentTimeNs;
    }

    private void applyVelocityMeasurement(Pose measuredVelocity, UpdateType updateType,
                                          double dt) {
        rawVelocity = measuredVelocity;
        rawAcceleration = rawVelocity.minus(prevRawVelocity).div(dt);
        applyFilterState(updateType);
        prevRawVelocity = rawVelocity;
    }

    private void applyFilterState(UpdateType updateType) {

        // Update sample size if using LowPass
        if (FILTER_WINDOW_SIZE != lastSize
                && velocityFilterMode == VelocityFilterMode.MOVING_AVERAGE) {
            applyMovingAverageWindow();
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

        if (updateType == UpdateType.BOTH || updateType == UpdateType.ACCELERATION) {
            acceleration = new Pose(
                    new Vector(Dist.fromIn(xState.rate()), Dist.fromIn(yState.rate())),
                    Angle.fromRad(headingState.rate())
            );
        }

    }

    /** Clears motion history after an odometry pose reset. */
    protected void resetKinematicEstimate(Pose newPose) {
        pose = newPose;
        velocity = Pose.zero();
        rawVelocity = Pose.zero();
        acceleration = Pose.zero();
        rawAcceleration = Pose.zero();
        prevPose = newPose;
        prevRawVelocity = Pose.zero();
        prevTimeNs = -1;
        xFilter.reset();
        yFilter.reset();
        headingFilter.reset();
    }

    public void setIsTuning(boolean isTuning) {
        setKalmanAutoTuning(isTuning, isTuning, isTuning);
    }

    /** Enables one-time calibration independently for translation, strafe, and heading. */
    public void setKalmanAutoTuning(boolean tuneX, boolean tuneY, boolean tuneHeading) {
        if (velocityFilterMode != VelocityFilterMode.ADAPTIVE_KALMAN) {
            isTuning = false;
            return;
        }
        ((AdaptiveKalmanFilter) xFilter).setAutoTuning(tuneX);
        ((AdaptiveKalmanFilter) yFilter).setAutoTuning(tuneY);
        ((AdaptiveKalmanFilter) headingFilter).setAutoTuning(tuneHeading);
        isTuning = tuneX || tuneY || tuneHeading;
    }

    public boolean isTuningVelocityFilter() { return isTuning; }

    // TUNING INJECTION/EXTRACTION

    public AdaptiveKalmanFilter.KalmanTuning getXKalmanTuning() {
        return kalman(xFilter) != null ? kalman(xFilter).getTuning() : null;
    }

    public void setXKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(xFilter) != null && tuning != null) {
            kalman(xFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getYKalmanTuning() {
        return kalman(yFilter) != null ? kalman(yFilter).getTuning() : null;
    }

    public void setYKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(yFilter) != null && tuning != null) {
            kalman(yFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getHeadingKalmanTuning() {
        return kalman(headingFilter) != null ? kalman(headingFilter).getTuning() : null;
    }

    public void setHeadingKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(headingFilter) != null && tuning != null) {
            kalman(headingFilter).setTuning(tuning);
        }
    }

    private AdaptiveKalmanFilter kalman(DataFilter filter) {
        return filter instanceof AdaptiveKalmanFilter ? (AdaptiveKalmanFilter) filter : null;
    }

    private void applyMovingAverageWindow() {
        ((LowPassFilter) xFilter).setSampleSize(FILTER_WINDOW_SIZE);
        ((LowPassFilter) yFilter).setSampleSize(FILTER_WINDOW_SIZE);
        ((LowPassFilter) headingFilter).setSampleSize(FILTER_WINDOW_SIZE);
        lastSize = FILTER_WINDOW_SIZE;
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
