package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PDSRoutineTest {
    @Test
    public void usesLargerClosedLoopTrialDeltas() {
        assertEquals(Math.toRadians(60.0),
                PDSRoutine.trialMagnitudeFor(PDSRoutine.Axis.HEADING), 1e-9);
        assertEquals(24.0,
                PDSRoutine.trialMagnitudeFor(PDSRoutine.Axis.DRIVE), 1e-9);
    }

    @Test
    public void timeWeightedSquaredErrorPenalizesLateErrorMore() {
        double early = PDSRoutine.accumulateTimeWeightedSquaredError(
                0.0, 1.0, 2.0, 0.1);
        double late = PDSRoutine.accumulateTimeWeightedSquaredError(
                0.0, 3.0, 2.0, 0.1);

        assertEquals(0.4, early, 1e-9);
        assertEquals(1.2, late, 1e-9);
        assertTrue(late > early);
    }

    @Test
    public void normalizedFiniteDifferenceUsesEachGainRange() {
        double pGradient = PDSRoutine.normalizedFiniteDifference(
                12.0, 8.0, 0.60, 0.40, 0.0, 1.0, 10.0);
        double dGradient = PDSRoutine.normalizedFiniteDifference(
                12.0, 8.0, 0.060, 0.040, 0.0, 0.10, 10.0);

        assertEquals(2.0, pGradient, 1e-9);
        assertEquals(2.0, dGradient, 1e-9);
    }

    @Test
    public void normalizedGradientStepScalesDifferentGainMagnitudesEqually() {
        double magnitude = Math.sqrt(2.0);
        double p = PDSRoutine.gradientUpdatedGain(
                0.50, 1.0, magnitude, 0.10, 0.0, 1.0);
        double d = PDSRoutine.gradientUpdatedGain(
                0.050, 1.0, magnitude, 0.10, 0.0, 0.10);

        assertEquals(0.4292893219, p, 1e-9);
        assertEquals(0.04292893219, d, 1e-9);
    }

    @Test
    public void gradientStepClampsToSafeBounds() {
        assertEquals(0.0, PDSRoutine.gradientUpdatedGain(
                0.05, 10.0, 10.0, 0.20, 0.0, 1.0), 1e-9);
        assertEquals(1.0, PDSRoutine.gradientUpdatedGain(
                0.95, -10.0, 10.0, 0.20, 0.0, 1.0), 1e-9);
    }

    @Test
    public void finiteDifferenceProbeIsCentralWhenThereIsRoom() {
        assertEquals(0.55,
                PDSRoutine.perturbGain(0.50, 0.05, 0.0, 1.0, 1), 1e-9);
        assertEquals(0.45,
                PDSRoutine.perturbGain(0.50, 0.05, 0.0, 1.0, -1), 1e-9);
    }

    @Test
    public void finiteDifferenceProbeFallsBackToOneSidedAtBound() {
        assertEquals(0.05,
                PDSRoutine.perturbGain(0.0, 0.05, 0.0, 1.0, 1), 1e-9);
        assertEquals(0.0,
                PDSRoutine.perturbGain(0.0, 0.05, 0.0, 1.0, -1), 1e-9);
    }

    @Test
    public void worseUpdateHalvesLearningRateWithoutPassingFloor() {
        assertEquals(0.04, PDSRoutine.reducedLearningRate(0.08, 0.005), 1e-9);
        assertEquals(0.005, PDSRoutine.reducedLearningRate(0.006, 0.005), 1e-9);
    }

    @Test
    public void relativeImprovementRejectsWorseCost() {
        assertEquals(0.20, PDSRoutine.relativeImprovement(10.0, 8.0), 1e-9);
        assertEquals(0.0, PDSRoutine.relativeImprovement(10.0, 12.0), 1e-9);
    }

    @Test
    public void stalledOperatorTestClearsSoftStaticFrictionDeadband() {
        double tolerance = Math.toRadians(2.5);

        assertEquals(-0.28125, PDSRoutine.ensureTestBreakawayPower(
                -0.2359652517928235,
                -0.043727866438586505,
                0.0,
                0.23125,
                tolerance,
                0.10), 1e-9);
    }

    @Test
    public void operatorTestBreakawayFloorIsLimitedToStallOutsideTolerance() {
        double tolerance = Math.toRadians(2.5);

        assertEquals(-0.20, PDSRoutine.ensureTestBreakawayPower(
                -0.20, -Math.toRadians(2.0), 0.0, 0.23125, tolerance, 0.10), 1e-9);
        assertEquals(-0.20, PDSRoutine.ensureTestBreakawayPower(
                -0.20, -Math.toRadians(3.0), -0.20, 0.23125, tolerance, 0.10), 1e-9);
        assertEquals(-0.40, PDSRoutine.ensureTestBreakawayPower(
                -0.40, -Math.toRadians(3.0), 0.0, 0.23125, tolerance, 0.10), 1e-9);
    }

    @Test
    public void bidirectionalPdsTestsStageAtFieldCenter() {
        for (PDSRoutine.Axis axis : PDSRoutine.Axis.values()) {
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getX().getIn(), 1e-9);
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getY().getIn(), 1e-9);
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getHeading().getRad(), 1e-9);
        }
    }

    @Test
    public void automaticPdsTrialsAlternateForwardAndBackward() {
        double magnitude = PDSRoutine.trialMagnitudeFor(PDSRoutine.Axis.DRIVE);

        double backward = PDSRoutine.alternatingTrialTarget(magnitude, magnitude);
        double forward = PDSRoutine.alternatingTrialTarget(backward, magnitude);

        assertEquals(-magnitude, backward, 1e-9);
        assertEquals(magnitude, forward, 1e-9);
    }
}
