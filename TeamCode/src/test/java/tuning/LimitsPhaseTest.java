package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayDeque;
import java.util.Deque;

public class LimitsPhaseTest {
    @Test
    public void percentileDoesNotAllowSingleSpikeToDefineAcceleration() {
        double[] acceleration = {
                9.7, 10.0, 10.1, 9.9, 10.2, 10.0, 9.8, 10.1, 10.0, 9.9,
                10.0, 9.8, 10.1, 9.9, 10.0, 10.2, 9.8, 10.1, 9.9, 10.0, 2000.0
        };

        assertEquals(10.2, LimitsPhase.percentile(acceleration, 0.95), 1e-9);
        assertTrue(LimitsPhase.percentile(acceleration, 0.95) < 2000.0);
    }

    @Test
    public void stableVelocityWindowIsDetectedButContinuedAccelerationIsNot() {
        Deque<LimitsPhase.TimedSample> stable = new ArrayDeque<>();
        Deque<LimitsPhase.TimedSample> rising = new ArrayDeque<>();
        for (int i = 0; i < 15; i++) {
            double time = i * 0.02;
            stable.addLast(new LimitsPhase.TimedSample(time, 48.0 + (i % 3 - 1) * 0.05));
            rising.addLast(new LimitsPhase.TimedSample(time, 20.0 + i));
        }

        assertTrue(LimitsPhase.isVelocityPlateau(stable, false));
        assertFalse(LimitsPhase.isVelocityPlateau(rising, false));
    }
}
