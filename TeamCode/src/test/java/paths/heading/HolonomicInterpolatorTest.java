package paths.heading;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Vector;

public class HolonomicInterpolatorTest {
    @Test
    public void backwardTangentDoesNotAddTerminalHalfTurn() {
        HolonomicInterpolator interpolator = new HolonomicInterpolator(
                InterpolationStyle.TANGENT_BACKWARD,
                Angle.zero(), Angle.zero(), null, null
        );
        interpolator.setPathLength(64.0);
        Vector west = Vector.of(-1.0, 0.0, DistUnit.IN);

        Angle targetAtEnd = interpolator.getHeadingTarg(0.0, west, west);

        assertEquals(0.0, Angle.wrap(targetAtEnd.getRad()), 1e-9);
    }

    @Test
    public void backwardTangentRetainsCurvatureDerivatives() {
        HolonomicInterpolator interpolator = new HolonomicInterpolator(
                InterpolationStyle.TANGENT_BACKWARD,
                Angle.zero(), Angle.zero(), null, null
        );
        interpolator.setPathLength(64.0);
        Vector west = Vector.of(-1.0, 0.0, DistUnit.IN);

        assertEquals(0.04,
                interpolator.getHeadingFirstDerivative(20.0, 0.04, west), 1e-9);
        assertEquals(-0.003,
                interpolator.getHeadingSecondDerivative(20.0, -0.003, west), 1e-9);
    }
}
