package controllers;

import feedforward.MotionParameters;

/**
 * Executes quick and displacement-profiled point turns.
 *
 * <p>Quick turns and overshoot recovery use the complete heading PDS controller. Normal profiled
 * motion deliberately uses only angular feedforward, the PDS controller's tuned static term, and
 * explicit angular velocity feedback.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TurnController {
    private static final double EPSILON = 1e-6;
    private static final double STALLED_ANGULAR_VELOCITY = 0.05;
    private static final double BREAKAWAY_RESERVE = 0.02;

    private final PDSController headingPds;
    private double angularKV;
    private double angularKA;
    private double angularVelocityFeedbackGain;

    private boolean overshootRecovery;

    public TurnController(PDSController.PDSCoefficients headingCoefficients,
                          double angularKV, double angularKA,
                          double angularVelocityFeedbackGain) {
        headingPds = new PDSController(headingCoefficients);
        headingPds.setAngularController();

        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
    }

    public void setCoefficients(PDSController.PDSCoefficients coefficients) {
        headingPds.setCoefficients(coefficients);
        reset();
    }

    public void setMotionGains(double angularKV, double angularKA,
                               double angularVelocityFeedbackGain) {
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        reset();
    }

    /** Uses the complete heading PDS for an unprofiled turn. */
    public double calculateQuick(double headingError) { return headingPds.calculate(headingError); }

    /**
     * Calculates a profiled turn command and permanently switches to PDS recovery after overshoot.
     */
    public double calculateProfiled(double headingError, double intendedDirection,
                                    MotionParameters targets, double measuredAngularVelocity) {
        if (!overshootRecovery && intendedDirection != 0.0 &&
                intendedDirection * headingError < -EPSILON) {
            overshootRecovery = true;
            headingPds.reset();
        }

        if (overshootRecovery) {
            return headingPds.calculate(headingError);
        }

        double targetVelocity = targets.getAngularVel();
        double targetAcceleration = targets.getAngularAccel();

        // A displacement profile ends with both targets at zero. Without position recovery, a
        // turn that loses its last bit of momentum before the heading tolerance receives exactly
        // zero command forever. Use the tuned heading PDS to capture the endpoint; the velocity
        // feedback tuner excludes this zero-target tail from its score.
        if (Math.abs(targetVelocity) <= EPSILON &&
                Math.abs(targetAcceleration) <= EPSILON) {
            return clip(headingPds.calculate(headingError));
        }

        double motionSign = 0.0;
        if (Math.abs(targetVelocity) > EPSILON) {
            motionSign = Math.signum(targetVelocity);
        } else if (Math.abs(targetAcceleration) > EPSILON) {
            motionSign = Math.signum(targetAcceleration);
        }

        double feedforward = angularKV * targetVelocity + angularKA * targetAcceleration
                + headingPds.getCoefficients().kS * motionSign;
        double velocityFeedback = angularVelocityFeedbackGain
                * (targetVelocity - measuredAngularVelocity);

        double requestedPower = feedforward + velocityFeedback;
        return clip(ensureProfiledMotionBreakaway(
                requestedPower,
                targetVelocity,
                measuredAngularVelocity,
                headingPds.getCoefficients().kS
        ));
    }

    /**
     * Restores enough authority to restart a profiled turn if deceleration feedforward cancels
     * the velocity/static terms below breakaway while the profile still requests motion.
     */
    static double ensureProfiledMotionBreakaway(double requestedPower, double targetVelocity,
                                                 double measuredVelocity, double staticGain) {
        if (Math.abs(targetVelocity) <= EPSILON ||
                Math.abs(measuredVelocity) >= STALLED_ANGULAR_VELOCITY) {
            return requestedPower;
        }

        double minimumPower = Math.min(1.0, Math.abs(staticGain) + BREAKAWAY_RESERVE);
        if (Math.abs(requestedPower) >= minimumPower &&
                Math.signum(requestedPower) == Math.signum(targetVelocity)) {
            return requestedPower;
        }
        return Math.copySign(minimumPower, targetVelocity);
    }

    public void reset() {
        overshootRecovery = false;
        headingPds.reset();
    }

    private static double clip(double power) { return Math.max(-1.0, Math.min(1.0, power)); }
}
