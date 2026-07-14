package paths.heading;

import geometry.Angle;
import geometry.Vector;

/**
 * Defines the contract for calculating target heading profiles across different drivetrain
 * kinematics.
 *
 * @author DrPixelCat - 7842 alum
 */
public interface HeadingInterpolator {
    /** Calculates the target heading, f(s). */
    Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent);

    /** Calculates the first spatial derivative, f'(s), in rad/in. */
    double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent);

    /** Calculates the second spatial derivative, f''(s), in rad/in^2. */
    double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent);

    /** Sets the total path length, primarily used for terminal blending math. */
    void setPathLength(double lengthInches);
}