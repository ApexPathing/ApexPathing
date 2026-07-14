package feedforward.generators;

import core.FollowerConstants;
import drivetrains.Mecanum;
import drivetrains.Mecanum.DirectionalLut.DirectionalKinematics;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;
import geometry.DistUnit;

/**
 * Generates holonomic profiles for mecanum drives, including direction-specific limits.
 *
 * <p>
 * Unlike ideal swerve, mecanum does not have equal authority in every robot-relative direction.
 * The directional LUT scales velocity and acceleration costs so a diagonal/strafe-heavy segment
 * gets a lower limit than an efficient forward segment.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public class MecanumProfileGenerator extends BaseProfileGenerator {
    /** Avoids divide-by-zero and unstable comparisons near flat derivatives. */
    private static final double EPSILON = 1e-6;
    /** Number of binary-search steps used for local velocity ceilings. */
    private static final int VELOCITY_SEARCH_ITERATIONS = 20;

    /** Tuned physical and feedforward limits for the robot. */
    private final FollowerConstants config;
    /** Direction-aware velocity/acceleration model for mecanum translation. */
    private final Mecanum.DirectionalLut limitCalculator;

    /**
     * Creates a mecanum profile generator for a path.
     */
    public MecanumProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
        this.limitCalculator = new Mecanum.DirectionalLut(
                config.forwardVelLimitIn,
                config.forwardAccelLimitIn,
                config.strafeVelLimitIn,
                config.strafeAccelLimitIn
        );
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point,
                                                    Path path, double maxAngVel,
                                                    double maxAngAccel) {
        double s = point.getDistanceToEndIn();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle headingAtPoint = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, headingAtPoint);
        DirectionalKinematics normalK = limitCalculator.getKinematics(getNormalVector(point),
                headingAtPoint);
        // Mecanum limits depend on robot-relative direction, so tangent and normal loads differ.
        double maxPhysicalVel = dirK.maxVel;

        double effectiveAngVelLimit = Math.min(config.angularVelLimitRad, maxAngVel);
        double effectiveAngAccelLimit = Math.min(config.angularAccelLimitRad,
                maxAngAccel);

        // Angular velocity limit: |f' * v| <= omega_max.
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

        // Directional multipliers make the closed-form limit messy; binary search is cheap here.
        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, 0.0, kappa, fPrime, fDoublePrime, dirK, normalK) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    /**
     * Estimates normalized mecanum power for a local state.
     * <p>
     * Tangential and centripetal terms use different directional multipliers because the robot may
     * be efficient along the tangent and inefficient along the normal, or vice versa.
     */
    private double evaluatePower(double v, double a, double kappa, double fPrime,
                                 double fDoublePrime, DirectionalKinematics tangentKinematics,
                                 DirectionalKinematics normalKinematics) {
        // Apply the LUT as power cost multipliers instead of pretending strafe is as efficient.
        double boostedKV = config.translationalKV * tangentKinematics.velMultiplier;
        double boostedKA = config.translationalKA * tangentKinematics.accelMultiplier;
        double transPower = (v * boostedKV)
                + (a * boostedKA)
                + signedStatic(v, a, config.translationalCoeffs.kS);

        double latPower = Math.abs(
                (v * v * kappa) * config.Kcentripetal * normalKinematics.accelMultiplier
        );

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a);
        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);
        double rotPower =
                Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return transPower + latPower + rotPower;
    }

    /**
     * Evaluates drivetrain utilization for the segment ending at {@code current}.
     */
    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEndIn();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        Angle robotHeading = path.getInterpolator().getHeadingTarg(
                s, current.getFirstDerivative(), finalTangent
        );

        DirectionalKinematics dirK = limitCalculator.getKinematics(current.getFirstDerivative(),
                robotHeading);
        DirectionalKinematics normalK = limitCalculator.getKinematics(getNormalVector(current),
                robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward = (v * config.translationalKV * dirK.velMultiplier)
                + (a_t * config.translationalKA * dirK.accelMultiplier)
                + signedStatic(v, a_t, config.translationalCoeffs.kS);

        // Centripetal correction is a sideways force, so mecanum inefficiency applies here too.
        double pLateral = (v * v * kappa) * config.Kcentripetal * normalK.accelMultiplier;

        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = Math.abs(pLateral);
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pLateral + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    /**
     * @return maximum braking acceleration allowed at this path sample
     */
    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    /**
     * @return maximum positive acceleration allowed at this path sample
     */
    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path,
                                              double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    /**
     * Limits tangential acceleration so heading acceleration stays inside angular constraints.
     * <p>
     * The same chain-rule relationship applies here:
     * {@code alpha = f'' * v^2 + f' * a}. Directional mecanum acceleration then clamps the result
     * further through {@code dirK.maxAccel}.
     */
    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double s = point.getDistanceToEndIn();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle robotHeading = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalAccel = dirK.maxAccel;
        double effectiveAngAccelLimit = Math.min(config.angularAccelLimitRad,
                maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) {
            return 0.0;
        }

        double alphaBase = fDoublePrime * currentVel * currentVel;
        if (Math.abs(fPrime) < EPSILON) {
            // With no dtheta/ds term, tangential accel cannot reduce angular acceleration.
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
     * Returns a unit-ish normal direction for centripetal acceleration at this point.
     * <p>
     * The sign of curvature decides which side of the tangent points toward the curve center.
     */
    private Vector getNormalVector(PathPoint point) {
        double kappa = point.getSignedCurvature();
        if (Math.abs(kappa) < EPSILON) {
            return Vector.zero();
        }

        // Normal force points toward the curve center; sign follows signed curvature.
        double vx = point.getFirstDerivative().getX().getIn();
        double vy = point.getFirstDerivative().getY().getIn();
        if (kappa < 0.0) {
            return Vector.of(vy, -vx, DistUnit.IN);
        }
        return Vector.of(-vy, vx, DistUnit.IN);
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
}
