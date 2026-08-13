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
}
