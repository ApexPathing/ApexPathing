package paths.heading;

import geometry.Angle;
import geometry.CubicSpline1D;
import geometry.Vector;

/**
 * Calculates heading profiles allowing independent rotational control.
 *
 * @author DrPixelCat - 7842 alum
 */
public class HolonomicInterpolator implements HeadingInterpolator {
    private final InterpolationStyle style;
    private final Angle startHeading;
    private final Angle endHeading;
    private final Angle customOffset;
    private final CubicSpline1D headingSpline;

    private double pathLength = 1.0;
    private final double DEFAULT_BLEND_WINDOW_IN = 15.0;
    private double blendWindow = DEFAULT_BLEND_WINDOW_IN;

    public HolonomicInterpolator(InterpolationStyle style, Angle startHeading,
                                 Angle endHeading, Angle customOffset, CubicSpline1D spline) {
        this.style = style;
        this.startHeading = startHeading != null ? startHeading.copy() : null;
        this.endHeading = endHeading != null ? endHeading.copy() : null;
        this.customOffset = customOffset != null ? customOffset.copy() : null;
        this.headingSpline = spline;
    }

    @Override
    public void setPathLength(double length) { this.pathLength = Math.max(length, 1e-6); }

    public void setBlendWindow(double windowLengthInches) { this.blendWindow = windowLengthInches; }

    /**
     * Calculates the blend parameter u (0.0 to 1.0) at the END of the path. s is distance
     * remaining. When s > blendWindow, u = 0. When s = 0, u = 1.
     */
    private double getBlendU(double s) {
        if (blendWindow <= 1e-6 || s > blendWindow) return 0.0;
        return 1.0 - (s / blendWindow);
    }

    private boolean usesHeadingSpline() {
        return (style == InterpolationStyle.NODE_BASED ||
                style == InterpolationStyle.FACING_POINT) && headingSpline != null;
    }

    private Angle getBaseHeadingAtEnd(Vector finalTangent) {
        switch (style) {
            case CONSTANT_START_HEADING:
                return startHeading;
            case CONSTANT_END_HEADING:
            case SMOOTH_START_TO_END:
                return endHeading;
            case TANGENT_FORWARD:
                return finalTangent.getTheta();
            case TANGENT_CUSTOM:
                return finalTangent.getTheta().plus(customOffset);
            case NODE_BASED:
            case FACING_POINT:
                return Angle.fromRad(headingSpline.evaluate(1.0));
            default:
                throw new IllegalStateException("Unhandled HolonomicHeadingStyle");
        }
    }

    private double getTerminalErrorRad(Vector finalTangent) {
        Angle finalBaseHeading = getBaseHeadingAtEnd(finalTangent);
        return finalBaseHeading.getShortestAngleTo(endHeading).getRad();
    }

    // region Kinematic Evaluations

    @Override
    public Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent) {
        Angle baseHeading;
        double pctTraveled = (pathLength - s) / pathLength;

        switch (style) {
            case CONSTANT_START_HEADING:
                baseHeading = startHeading.copy();
                break;
            case CONSTANT_END_HEADING:
                baseHeading = endHeading.copy();
                break;
            case SMOOTH_START_TO_END:
                // Full-path cubic ease-in/ease-out interpolation
                double smoothU = (3.0 * pctTraveled * pctTraveled) - (2.0 * Math.pow(pctTraveled,
                        3));
                double diffRad = startHeading.getShortestAngleTo(endHeading).getRad();
                baseHeading = Angle.fromRad(startHeading.getRad() + (diffRad * smoothU));
                break;
            case TANGENT_FORWARD:
                baseHeading = pathTangent.getTheta();
                break;
            case TANGENT_CUSTOM:
                baseHeading = pathTangent.getTheta().plus(customOffset);
                break;
            case NODE_BASED:
            case FACING_POINT:
                baseHeading = Angle.fromRad(headingSpline.evaluate(pctTraveled));
                break;
            default:
                throw new IllegalStateException("Unhandled HolonomicHeadingStyle");
        }

        // Apply the 15-inch terminal blend if necessary
        double u = getBlendU(s);
        if (u > 0.0 && style != InterpolationStyle.SMOOTH_START_TO_END) {
            double terminalError = getTerminalErrorRad(finalTangent);
            double blendSmoothU = (3.0 * u * u) - (2.0 * u * u * u);
            return Angle.fromRad(baseHeading.getRad() + (terminalError * blendSmoothU));
        }

        return baseHeading;
    }

    @Override
    public double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent) {
        double basePrime = 0.0;
        double pctTraveled = (pathLength - s) / pathLength;

        if (style == InterpolationStyle.TANGENT_FORWARD || style == InterpolationStyle.TANGENT_CUSTOM) {
            basePrime = kappa;
        } else if (style == InterpolationStyle.SMOOTH_START_TO_END) {
            // Chain rule: d(theta)/ds_traveled = d(theta)/d(pct) * (1 / pathLength)
            double diffRad = startHeading.getShortestAngleTo(endHeading).getRad();
            basePrime =
                    diffRad * (6.0 * pctTraveled - 6.0 * pctTraveled * pctTraveled) / pathLength;
        } else if (usesHeadingSpline()) {
            basePrime = headingSpline.getFirstDerivative(pctTraveled) * (1.0 / pathLength);
        }

        double u = getBlendU(s);
        if (u > 0.0 && style != InterpolationStyle.SMOOTH_START_TO_END) {
            double terminalError = getTerminalErrorRad(finalTangent);
            double dSmoothU = (6.0 * u - 6.0 * u * u) / blendWindow;
            return basePrime + (terminalError * dSmoothU);
        }

        return basePrime;
    }

    @Override
    public double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent) {
        double baseDoublePrime = 0.0;
        double pctTraveled = (pathLength - s) / pathLength;

        if (style == InterpolationStyle.TANGENT_FORWARD || style == InterpolationStyle.TANGENT_CUSTOM) {
            baseDoublePrime = dKappa;
        } else if (style == InterpolationStyle.SMOOTH_START_TO_END) {
            // Chain rule: d2(theta)/ds_traveled2 = d2(theta)/d(pct)2 * (1 / pathLength)^2
            double diffRad = startHeading.getShortestAngleTo(endHeading).getRad();
            baseDoublePrime = diffRad * (6.0 - 12.0 * pctTraveled) / (pathLength * pathLength);
        } else if (usesHeadingSpline()) {
            baseDoublePrime =
                    headingSpline.getSecondDerivative(pctTraveled) * (1.0 / (pathLength * pathLength));
        }

        double u = getBlendU(s);
        if (u > 0.0 && style != InterpolationStyle.SMOOTH_START_TO_END) {
            double terminalError = getTerminalErrorRad(finalTangent);
            double d2SmoothU = (6.0 - 12.0 * u) / (blendWindow * blendWindow);
            return baseDoublePrime + (terminalError * d2SmoothU);
        }

        return baseDoublePrime;
    }

    // endregion
}
