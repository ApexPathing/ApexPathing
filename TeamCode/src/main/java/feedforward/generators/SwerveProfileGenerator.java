package feedforward.generators;

import core.FollowerConstants;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/**
 * Generates time-optimal motion profiles for Swerve drivetrains by evaluating
 * continuous voltage saturation constraints algebraically.
 *
 * <p>
 * Swerve can point its traction vector, so translational tangent and normal power combine like a
 * vector. Heading power is still added because steering the robot body consumes the same normalized
 * power budget.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class SwerveProfileGenerator extends BaseProfileGenerator {
    /** Avoids divide-by-zero and unstable comparisons near flat derivatives. */
    private static final double EPSILON = 1e-6;
    /** Number of binary-search steps used for local velocity ceilings. */
    private static final int VELOCITY_SEARCH_ITERATIONS = 8;

    /** Tuned physical and feedforward limits for the robot. */
    private final FollowerConstants constants;

    /**
     * Creates a swerve profile generator for a path.
     */
    public SwerveProfileGenerator(FollowerConstants constants, Path path) {
        super.path = path;
        this.constants = constants;
    }

    // region Base Pass (Velocity Ceiling)

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point, Path path,
                                                    double maxAngVel, double maxAngAccel) {
        double s = point.getDistanceToEndIn();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalVel = constants.forwardVelLimitIn;
        double effectiveAngVelLimit = Math.min(constants.angularVelLimitRad, maxAngVel);
        double effectiveAngAccelLimit = Math.min(constants.angularAccelLimitRad,
                maxAngAccel);

        double min_v = 0.0;
        double max_v = maxPhysicalVel;

        // Solve the velocity ceiling numerically because heading acceleration depends on v^2.
        // At zero tangential acceleration, alpha = f'' * v^2.
        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;
            double omega = fPrime * mid_v;
            double alpha = fDoublePrime * mid_v * mid_v;
            boolean violatesAngularLimit =
                    Math.abs(omega) > effectiveAngVelLimit + EPSILON ||
                            Math.abs(alpha) > effectiveAngAccelLimit + EPSILON;

            if (violatesAngularLimit || calculatePowerUtilization(mid_v, 0.0, path, point) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    // endregion

    // region Integration Passes (Acceleration / Deceleration)

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point,
                                              Path path, double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point,
                                           Path path, double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    // endregion

    // region Utility and Evaluation

    /**
     * Calculates the theoretical total motor power required to execute a given kinematic state.
     *
     * @param vel path-relative velocity
     * @param accel path-relative acceleration
     * @param path path being sampled
     * @param point sample to evaluate
     * @return normalized utilization, where 1.0 is full available power
     */
    protected double calculatePowerUtilization(double vel, double accel, Path path, PathPoint point) {
        EvaluationResult result = new EvaluationResult();
        evaluateState(path, point, vel, accel, result);
        return result.totalPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        evaluateState(path, current, v, a_t, outResult);
    }

    /**
     * Fills a utilization result for one swerve state.
     * <p>
     * Tangential power comes from speed and tangential acceleration. Normal power is the
     * centripetal term {@code v^2 * kappa}, scaled by the tuned centripetal coefficient. Heading
     * demand follows the chain-rule formulas {@code omega = f'v} and
     * {@code alpha = f''v^2 + f'a}.
     */
    private void evaluateState(Path path, PathPoint point, double vel, double accel,
                               EvaluationResult outResult) {
        double s = point.getDistanceToEndIn();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

        double tanPow = (vel * constants.translationalKV)
                + (accel * constants.translationalKA)
                + signedStatic(vel, accel, constants.translationalCoeffs.kS);

        double normPow = (vel * vel * kappa) * constants.Kcentripetal;

        double omega = fPrime * vel;
        double alpha = (fDoublePrime * (vel * vel)) + (fPrime * accel);

        double headingKs = signedStatic(omega, alpha, constants.headingCoeffs.kS);

        double heading = (omega * constants.angularKV)
                + (alpha * constants.angularKA)
                + headingKs;

        // Swerve can point the traction vector, so translation combines as vector magnitude.
        outResult.pForward = Math.abs(tanPow);
        outResult.pLateral = Math.abs(normPow);
        outResult.pHeading = Math.abs(heading);
        outResult.totalPower = Math.hypot(tanPow, normPow) + Math.abs(heading);
        outResult.maxUtilization = outResult.totalPower;
    }

    /**
     * Computes the tangential acceleration cap imposed by angular acceleration limits.
     * <p>
     * Rearranging {@code alpha = f'' * v^2 + f' * a} gives a legal interval for {@code a}. This
     * method returns the positive side for acceleration passes or the negative side for braking
     * passes, always clamped by the physical forward acceleration limit.
     */
    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double maxPhysicalAccel = constants.forwardAccelLimitIn;
        double effectiveAngAccelLimit = Math.min(constants.angularAccelLimitRad,
                maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) {
            return 0.0;
        }

        double s = point.getDistanceToEndIn();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);
        double alphaBase = fDoublePrime * currentVel * currentVel;

        if (Math.abs(fPrime) < EPSILON) {
            // Tangential accel cannot help if heading is not changing with path distance.
            return Math.abs(alphaBase) <= effectiveAngAccelLimit + EPSILON
                    ? maxPhysicalAccel : 0.0;
        }

        double boundA = (-effectiveAngAccelLimit - alphaBase) / fPrime;
        double boundB = (effectiveAngAccelLimit - alphaBase) / fPrime;
        double minAccel = Math.min(boundA, boundB);
        double maxAccel = Math.max(boundA, boundB);

        double angularLimitedAccel = positiveAccel ? maxAccel : -minAccel;
        return Math.min(maxPhysicalAccel, Math.max(0.0, angularLimitedAccel));
    }

    /**
     * Applies static friction in the direction implied by velocity, or acceleration from rest.
     */
    private double signedStatic(double velocity, double accel, double kS) {
        if (Math.abs(velocity) > EPSILON) {
            return Math.signum(velocity) * kS;
        }
        if (Math.abs(accel) > EPSILON) {
            return Math.signum(accel) * kS;
        }
        return 0.0;
    }

    // endregion
}
