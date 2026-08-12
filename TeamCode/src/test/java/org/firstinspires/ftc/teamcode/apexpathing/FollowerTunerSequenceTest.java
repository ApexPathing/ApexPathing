package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class FollowerTunerSequenceTest {
    @Test
    public void advancesThroughEveryTuningPhaseInOrder() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();

        for (int i = 0; i < phases.length - 1; i++) {
            assertEquals(phases[i + 1], FollowerTuner.nextPhase(phases[i]));
        }
        assertNull(FollowerTuner.nextPhase(phases[phases.length - 1]));
    }
}
