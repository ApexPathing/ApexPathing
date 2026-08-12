package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import controllers.PDSController.PDSCoefficients;

public class PDSRoutineTest {
    @Test
    public void zieglerNicholsPdUsesUltimateGainAndPeriod() {
        PDSCoefficients coefficients = PDSRoutine.calculateZieglerNicholsPd(6.25, 0.80);

        assertEquals(5.0, coefficients.kP, 1e-9);
        assertEquals(0.5, coefficients.kD, 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void zieglerNicholsRejectsNonPositiveIdentification() {
        PDSRoutine.calculateZieglerNicholsPd(0.0, 1.0);
    }

    @Test
    public void relayAnalyzerRejectsTransientCyclesAndRecoversStableOscillation() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.30, 0.10);
        double period = 1.20;
        double stableAmplitude = 0.75;
        double dt = 0.01;

        for (double time = 0.0; time <= 10.0; time += dt) {
            // Let the first two periods decay so the accepted tail must ignore startup behavior.
            double transientScale = time < 2.4 ? 1.0 + (2.4 - time) * 0.25 : 1.0;
            double position = stableAmplitude * transientScale *
                    Math.sin(2.0 * Math.PI * time / period);
            analyzer.observe(time, position);
        }

        assertTrue(analyzer.hasStableEstimate());
        RelayOscillationAnalyzer.Estimate estimate = analyzer.estimate();
        assertEquals(stableAmplitude, estimate.amplitude, 0.01);
        assertEquals(period, estimate.periodSeconds, 0.02);
        assertEquals(4.0 * 0.30 / (Math.PI * stableAmplitude),
                estimate.ultimateGain, 0.01);
    }

    @Test
    public void relayAnalyzerDoesNotAcceptInconsistentCycles() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.25, 0.08);
        double dt = 0.01;
        for (double time = 0.0; time <= 12.0; time += dt) {
            int cycle = (int) Math.floor(time);
            double amplitude = cycle % 2 == 0 ? 0.4 : 1.0;
            double position = amplitude * Math.sin(2.0 * Math.PI * time);
            analyzer.observe(time, position);
        }

        assertTrue(analyzer.getCycleCount() >= 6);
        assertTrue(!analyzer.hasStableEstimate());
    }
}
