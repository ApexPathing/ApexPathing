package feedforward.generators;

import core.FollowerConstants;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/**
 * Generates path profiles for differential/tank drives.
 *
 * <p>Tank drives spend the same left/right wheel voltage budget on forward motion and heading
 * motion. This generator therefore treats forward and angular power as additive normalized
 * utilization.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TankProfileGenerator extends BaseProfileGenerator {
    /** Number of binary-search steps used for velocity ceilings. */
    private static final int VELOCITY_SEARCH_ITERATIONS = 8;

    /** Creates a tank profile generator for a path. */
    public TankProfileGenerator(FollowerConstants constants, Path path) {
        super(constants, path);
    }

    /**
     * Computes the maximum path speed at one sample.
     * 
     * <p>The heading interpolator supplies {@code dtheta/ds} and {@code d2theta/ds2}. Those convert
     * path speed into heading speed and acceleration: {@code omega = f' * v} and 
     * {@code alpha = f'' * v^2} when tangential acceleration is zero.
     */
    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point,
                                                    Path path, double maxAngVel,
                                                    double maxAngAccel) {
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

        // Angular velocity limit: |f' * v| <= omega_max, so v <= omega_max / |f'|.
        if (Math.abs(fPrime) > EPSILON) {
            double maxVelFromOmega = effectiveAngVelLimit / Math.abs(fPrime);
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromOmega);
        }

        // Angular acceleration limit at zero tangential accel: |f'' * v^2| <= alpha_max.
        if (Math.abs(fDoublePrime) > EPSILON) {
            double maxVelFromAlpha = Math.sqrt(effectiveAngAccelLimit / Math.abs(fDoublePrime));
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromAlpha);
        }

        double min_v = 0.0;
        double max_v = maxPhysicalVel;

        // Rotation power depends on the path heading derivatives, so search the usable top speed.
        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, fPrime, fDoublePrime) > 1.0) { max_v = mid_v; }
            else { min_v = mid_v; }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    /**
     * Estimates normalized tank power for a local state.
     *
     * <p>Translation uses {@code kV*v + kA*a + kS}. Heading uses the same structure with
     * {@code omega} and {@code alpha}. The two absolute magnitudes are added because they share
     * the same motor output budget.
     */
    private double evaluatePower(double v, double fPrime, double fDoublePrime) {
        double transPower = Math.abs(v * constants.translationalKV +
                        signedStatic(v, 0.0, constants.translationalCoeffs.kS));

        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v) + fPrime * 0.0;
        double headingKs = signedStatic(omega, alpha, constants.headingCoeffs.kS);

        double rotPower = Math.abs(omega * constants.angularKV + alpha *
                constants.angularKA + headingKs);

        return transPower + rotPower;
    }

    /** Evaluates the final power/utilization at a path segment. */
    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEndIn();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        // Tank translation and rotation share the same drivetrain output budget.
        double omega = fPrime * v;
        double alpha = fDoublePrime * (v * v) + fPrime * a_t;

        double pForward = v * constants.translationalKV + a_t * constants.translationalKA
                        + signedStatic(v, a_t, constants.translationalCoeffs.kS);

        double headingKs = signedStatic(omega, alpha, constants.headingCoeffs.kS);
        double pHeading = omega * constants.angularKV + alpha * constants.angularKA + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = 0.0;
        outResult.pHeading = Math.abs(pHeading);
        outResult.totalPower = outResult.pForward + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path,
                                              double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    /**
     * Limits tangential acceleration so heading acceleration stays within bounds.
     *
     * <p>Since {@code alpha = f'' * v^2 + f' * a}, solving {@code -alphaMax <= alpha <= alphaMax}
     * gives an allowed interval for {@code a}. The caller asks for either the positive side
     * (accelerating) or the negative side (braking).
     */
    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double s = point.getDistanceToEndIn();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalAccel = constants.forwardAccelLimitIn;
        double effectiveAngAccelLimit = Math.min(constants.angularAccelLimitRad, maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) { return 0.0; }

        double alphaBase = fDoublePrime * currentVel * currentVel;
        if (Math.abs(fPrime) < EPSILON) {
            // If heading does not depend on distance here, accel cannot trade against alpha.
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
}