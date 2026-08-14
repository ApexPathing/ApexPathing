package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import tuning.VelocityFeedbackPhase;

public class FollowerTunerSequenceTest {
    @Test
    public void advancesThroughEveryTuningPhaseInOrder() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();

        for (int i = 0; i < phases.length - 1; i++) {
            assertEquals(phases[i + 1], FollowerTuner.nextPhase(phases[i]));
        }
        assertNull(FollowerTuner.nextPhase(phases[phases.length - 1]));
    }

    @Test
    public void centripetalCompletionConstructsVelocityFeedbackNext() {
        assertEquals(VelocityFeedbackPhase.class,
                FollowerTuner.nextPhaseClass(FollowerTuner.Phase.CENTRIPETAL));
    }

    @Test
    public void velocityFeedbackRequiresBothAxesToBeTuned() {
        assertFalse(FollowerTuner.velocityFeedbackTuned(0.0, 0.25));
        assertFalse(FollowerTuner.velocityFeedbackTuned(0.10, 0.0));
        assertTrue(FollowerTuner.velocityFeedbackTuned(0.10, 0.25));
    }

    @Test
    public void everyPhaseCanBeSelectedRegardlessOfPriorCompletion() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();
        boolean[] original = new boolean[phases.length];
        for (int i = 0; i < phases.length; i++) {
            original[i] = phases[i].tuned;
            phases[i].tuned = false;
        }

        try {
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.VELOCITY_FEEDBACK));

            FollowerTuner.Phase.HEADING.tuned = true;
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
        } finally {
            for (int i = 0; i < phases.length; i++) { phases[i].tuned = original[i]; }
        }
    }
}
