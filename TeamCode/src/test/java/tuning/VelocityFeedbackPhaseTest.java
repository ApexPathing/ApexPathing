package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void translationScoringExcludesAccelerationAndEndpointCaptureRegions() {
        assertFalse(VelocityFeedbackPhase.isUsableTranslationSample(20.0, 2.0, 48.0));
        assertTrue(VelocityFeedbackPhase.isUsableTranslationSample(20.0, 24.0, 48.0));
        assertFalse(VelocityFeedbackPhase.isUsableTranslationSample(20.0, 47.0, 48.0));
        assertFalse(VelocityFeedbackPhase.isUsableTranslationSample(0.5, 24.0, 48.0));
    }

    @Test
    public void automaticCandidateRequiresHeldOutImprovementAndDirectionalAgreement() {
        VelocityFeedbackPhase.CandidateResult incumbent = result(1.0, 1.0, 0.0);
        assertTrue(VelocityFeedbackPhase.acceptsCandidate(
                incumbent, result(0.90, 0.94, 0.0)));
        assertFalse(VelocityFeedbackPhase.acceptsCandidate(
                incumbent, result(0.70, 0.97, 0.0)));
        assertFalse(VelocityFeedbackPhase.acceptsCandidate(
                incumbent, result(0.60, 0.90, 0.0)));
    }

    @Test
    public void automaticCandidateRejectsSaturationDependentImprovement() {
        assertFalse(VelocityFeedbackPhase.acceptsCandidate(
                result(1.0, 1.0, 0.0), result(0.85, 0.90, 0.25)));
    }

    private static VelocityFeedbackPhase.CandidateResult result(
            double outbound, double returning, double saturation) {
        return new VelocityFeedbackPhase.CandidateResult(
                outbound, returning, 0.0, 0.0, 1.0, saturation);
    }
}
