package tuning;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HeadingPhaseTest {
    @Test
    public void manualTargetsAlternateAcrossOneSixtyDegreeMove() {
        assertEquals(0.0, HeadingPhase.nextManualTestTarget(60.0), 0.0);
        assertEquals(60.0, HeadingPhase.nextManualTestTarget(0.0), 0.0);
    }
}
