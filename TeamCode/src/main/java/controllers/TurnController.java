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
    // Preserve breakaway authority through the low-speed end of profile deceleration. Waiting
    // until nearly zero lets static friction stop the robot for a loop before recovery restarts it.
    private static final double LOW_SPEED_ANGULAR_VELOCITY = 0.25;
    private static final double ENDPOINT_CAPTURE_HEADING = Math.toRadians(10.0);
    private static final double ENDPOINT_CAPTURE_MEASURED_VELOCITY = 1.0;
    private static final double BREAKAWAY_RESERVE = 0.02;
    // Velocity feedback is a correction, not the primary command. Bounding its contribution keeps
    // an over-tuned/stale gain from turning profile tracking into full-power bang-bang control.
    private static final double MAX_ACCELERATING_FEEDBACK_POWER = 0.50;
    private static final double MAX_BRAKING_FEEDBACK_POWER = 0.25;

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

        double motionSign = 0.0;
        if (Math.abs(targetVelocity) > EPSILON) {
            motionSign = Math.signum(targetVelocity);
        } else if (Math.abs(targetAcceleration) > EPSILON) {
            motionSign = Math.signum(targetAcceleration);
        }

        double feedforward = angularKV * targetVelocity + angularKA * targetAcceleration
                + headingPds.getCoefficients().kS * motionSign;
        double velocityFeedback = clipVelocityFeedback(angularVelocityFeedbackGain
                * (targetVelocity - measuredAngularVelocity), intendedDirection);

        double requestedPower = feedforward + velocityFeedback;
        // Endpoint capture is based on actual remaining heading and motion, not a particular LUT
        // row. This keeps the handoff continuous even when displacement advances in coarse steps
        // or the chassis stops before the profile reaches its exact zero-velocity sample.
        if (Math.abs(headingError) < ENDPOINT_CAPTURE_HEADING) {
            double positionPower = headingPds.calculate(headingError);
            requestedPower = blendEndpointCapturePower(
                    requestedPower, positionPower, headingError,
                    measuredAngularVelocity, ENDPOINT_CAPTURE_HEADING,
                    ENDPOINT_CAPTURE_MEASURED_VELOCITY);
        }
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
                Math.abs(measuredVelocity) >= LOW_SPEED_ANGULAR_VELOCITY) {
            return requestedPower;
        }

        double minimumPower = Math.min(1.0, Math.abs(staticGain) + BREAKAWAY_RESERVE);
        if (Math.abs(requestedPower) >= minimumPower &&
                Math.signum(requestedPower) == Math.signum(targetVelocity)) {
            return requestedPower;
        }
        return Math.copySign(minimumPower, targetVelocity);
    }

    static double blendEndpointCapturePower(double profilePower, double positionPower,
                                            double headingError, double measuredVelocity,
                                            double captureHeading, double stalledVelocity) {
        if (!Double.isFinite(profilePower) || !Double.isFinite(positionPower) ||
                !Double.isFinite(headingError) || !Double.isFinite(measuredVelocity) ||
                !Double.isFinite(captureHeading) || !Double.isFinite(stalledVelocity) ||
                captureHeading <= 0.0 || stalledVelocity <= 0.0) {
            return profilePower;
        }
        if (Math.abs(headingError) >= captureHeading) { return profilePower; }
        double headingWeight = 1.0 - Math.abs(headingError) / captureHeading;
        double stalledWeight = Math.max(0.0,
                1.0 - Math.abs(measuredVelocity) / stalledVelocity);
        // Do not let position-controller derivative braking fight a still-active high-speed
        // profile. It takes authority progressively only as both angle and actual motion settle.
        double positionWeight = headingWeight * stalledWeight;
        return profilePower * (1.0 - positionWeight) + positionPower * positionWeight;
    }

    public void reset() {
        overshootRecovery = false;
        headingPds.reset();
    }

    private static double clip(double power) { return Math.max(-1.0, Math.min(1.0, power)); }

    private static double clipVelocityFeedback(double feedback, double intendedDirection) {
        if (Math.abs(intendedDirection) <= EPSILON) {
            return Math.max(-MAX_BRAKING_FEEDBACK_POWER,
                    Math.min(MAX_BRAKING_FEEDBACK_POWER, feedback));
        }
        double alongDirection = feedback * Math.signum(intendedDirection);
        double bounded = Math.max(-MAX_BRAKING_FEEDBACK_POWER,
                Math.min(MAX_ACCELERATING_FEEDBACK_POWER, alongDirection));
        return bounded * Math.signum(intendedDirection);
    }
}
