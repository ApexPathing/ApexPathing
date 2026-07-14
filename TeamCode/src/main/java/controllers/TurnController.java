package controllers;

import feedforward.MotionParameters;

/**
 * Executes quick and displacement-profiled point turns.
 *
 * <p>
 * Quick turns and overshoot recovery use the complete heading PDS controller. Normal profiled
 * motion deliberately uses only angular feedforward, the PDS controller's tuned static term, and
 * explicit angular velocity feedback.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class TurnController {
    private static final double EPSILON = 1e-6;

    private final PDSController headingPds;
    private final double angularKV;
    private final double angularKA;
    private final double angularVelocityFeedbackGain;

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

    /** Uses the complete heading PDS for an unprofiled turn. */
    public double calculateQuick(double headingError) {
        return headingPds.calculate(headingError);
    }

    /**
     * Calculates a profiled turn command and permanently switches to PDS recovery after overshoot.
     */
    public double calculateProfiled(double headingError, double intendedDirection,
                                    MotionParameters targets, double measuredAngularVelocity) {
        if (!overshootRecovery && intendedDirection != 0.0 &&
                (intendedDirection * headingError) < -EPSILON) {
            overshootRecovery = true;
            headingPds.reset();
        }

        if (overshootRecovery) {
            return headingPds.calculate(headingError);
        }

        double targetVelocity = targets.getAngularVel();
        double targetAcceleration = targets.getAngularAccel();
        double motionSign = Math.abs(targetVelocity) > EPSILON
                ? Math.signum(targetVelocity)
                : (Math.abs(targetAcceleration) > EPSILON
                ? Math.signum(targetAcceleration) : 0.0);

        double feedforward = (angularKV * targetVelocity)
                + (angularKA * targetAcceleration)
                + (headingPds.getCoefficients().kS * motionSign);
        double velocityFeedback = angularVelocityFeedbackGain
                * (targetVelocity - measuredAngularVelocity);

        return clip(feedforward + velocityFeedback);
    }

    public void reset() {
        overshootRecovery = false;
        headingPds.reset();
    }

    private static double clip(double power) { return Math.max(-1.0, Math.min(1.0, power)); }
}
