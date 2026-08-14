package tuning;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class CentripetalPhaseTest {
    @Test
    public void turnsAroundNearEndpointWithoutWaitingForFullPoseSettling() {
        assertTrue(CentripetalPhase.readyForTurnaround(0.99, 1.0, 4.0));
    }

    @Test
    public void doesNotTurnAroundEarlyFarAwayOrAtHighSpeed() {
        assertFalse(CentripetalPhase.readyForTurnaround(0.95, 1.0, 4.0));
        assertFalse(CentripetalPhase.readyForTurnaround(0.99, 2.0, 4.0));
        assertFalse(CentripetalPhase.readyForTurnaround(0.99, 1.0, 8.0));
        assertFalse(CentripetalPhase.readyForTurnaround(Double.NaN, 1.0, 4.0));
    }
}
