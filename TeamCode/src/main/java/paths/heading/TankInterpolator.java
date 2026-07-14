package paths.heading;

import geometry.Angle;
import geometry.Vector;

/**
 * Calculates heading profiles strictly constrained to the path tangent.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TankInterpolator implements HeadingInterpolator {

    private final InterpolationStyle style;

    public TankInterpolator(InterpolationStyle style) {
        if (style == InterpolationStyle.TANGENT_OPTIMAL) {
            throw new IllegalArgumentException("TANGENT_OPTIMAL must be resolved to FORWARD or " +
                    "BACKWARD before runtime instantiation.");
        }
        this.style = style;
    }

    @Override
    public void setPathLength(double lengthInches) {
        // Unused for tank kinematics
    }

    @Override
    public Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent) {
        if (style == InterpolationStyle.TANGENT_BACKWARD) {
            return pathTangent.getTheta().plus(Angle.fromRad(Math.PI));
        }
        return pathTangent.getTheta();
    }

    @Override
    public double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent) {
        // Whether driving forward or backward, the spatial rate of change is just the curvature
        return kappa;
    }

    @Override
    public double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent) {
        return dKappa;
    }
}