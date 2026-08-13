package tuning;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import geometry.Angle;
import geometry.Pose;
import paths.movements.Turn;

public class VelocityFeedbackPhaseTest {
    @Test
    public void returnTurnProgressIsPositiveInItsIntendedDirection() {
        Turn clockwiseReturn = new Turn(
                new Pose(Pose.zero().getVec(), Angle.fromDeg(90.0)),
                Angle.zero()
        );

        assertEquals(Math.toRadians(45.0),
                VelocityFeedbackPhase.turnProfileProgress(
                        clockwiseReturn, Angle.fromDeg(45.0)), 1e-9);
        assertEquals(Math.toRadians(90.0),
                VelocityFeedbackPhase.turnProfileProgress(
                        clockwiseReturn, Angle.zero()), 1e-9);
    }
}
