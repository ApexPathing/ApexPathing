package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
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

    @Test
    public void relayAnalyzerToleratesOneIsolatedPeriodOutlier() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.30, 0.10);
        double[] periods = { 1.2, 1.2, 0.45, 1.2, 1.2, 1.2, 1.2 };
        double time = 0.0;
        double dt = 0.005;

        for (double period : periods) {
            double cycleEnd = time + period;
            while (time < cycleEnd) {
                double phase = (time - (cycleEnd - period)) / period;
                analyzer.observe(time, 0.75 * Math.sin(2.0 * Math.PI * phase));
                time += dt;
            }
        }

        assertTrue(analyzer.getCycleCount() >= 6);
        assertTrue(analyzer.hasStableEstimate());
        assertEquals(1.2, analyzer.estimate().periodSeconds, 0.03);
    }

    @Test
    public void relayDeadlineScalesWithObservedSlowCycles() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.30, 0.10);
        double period = 5.2;
        double dt = 0.01;
        for (double time = 0.0; time <= 13.0; time += dt) {
            analyzer.observe(time, 0.20 * Math.sin(2.0 * Math.PI * time / period));
        }

        assertTrue(analyzer.getCycleCount() >= 2);
        assertTrue(analyzer.recommendedTimeoutSeconds(16.0, 45.0) > 30.0);
        assertTrue(analyzer.recommendedTimeoutSeconds(16.0, 45.0) <= 45.0);
    }

    @Test
    public void relayDeadlineExtendsAfterFirstSlowCycle() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.36, 1.50);

        // Reproduce the timing pattern from a slow translational FTCodeSim relay run. The second
        // complete cycle would arrive after the old fixed 16-second deadline.
        analyzer.observe(2.16, 1.51);
        analyzer.observe(6.49, -1.51);
        analyzer.observe(10.82, 1.51);

        assertEquals(1, analyzer.getCycleCount());
        assertTrue(analyzer.recommendedTimeoutSeconds(16.0, 60.0) > 50.0);
        assertTrue(analyzer.recommendedTimeoutSeconds(16.0, 60.0) <= 60.0);
    }

    @Test
    public void translationalRelayHasReserveAboveBreakawayPower() {
        double staticGain = 0.24375;

        assertEquals(0.46375,
                PDSRoutine.relayPowerFor(PDSRoutine.Axis.DRIVE, staticGain), 1e-9);
        assertEquals(0.46375,
                PDSRoutine.relayPowerFor(PDSRoutine.Axis.STRAFE, staticGain), 1e-9);
        assertTrue(PDSRoutine.relayPowerFor(PDSRoutine.Axis.DRIVE, staticGain) >=
                staticGain + 0.20);
    }

    @Test
    public void unstableFullWindowGetsTimeForOutlierToAgeOut() {
        RelayOscillationAnalyzer analyzer = new RelayOscillationAnalyzer(0.40, 0.07);
        double[] recordedPeriods = {
                2.3171296, 2.7425875, 3.3386255, 3.1592657, 3.6069202, 1.8639204
        };
        double time = 0.0;
        analyzer.observe(time, 0.08);
        for (double period : recordedPeriods) {
            analyzer.observe(time + period / 2.0, -0.08);
            time += period;
            analyzer.observe(time, 0.08);
        }

        assertEquals(6, analyzer.getCycleCount());
        assertFalse(analyzer.hasStableEstimate());
        assertTrue(analyzer.recommendedTimeoutSeconds(16.0, 45.0) >
                time + 2.0 * 3.6069202);

        double recoveryPeriod = 3.40;
        analyzer.observe(time + recoveryPeriod / 2.0, -0.08);
        time += recoveryPeriod;
        analyzer.observe(time, 0.08);
        assertTrue(analyzer.hasStableEstimate());
    }

    @Test
    public void bidirectionalPdsTestsStageAtFieldCenter() {
        for (PDSRoutine.Axis axis : PDSRoutine.Axis.values()) {
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getX().getIn(), 1e-9);
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getY().getIn(), 1e-9);
            assertEquals(0.0, PDSRoutine.stagingPoseFor(axis).getHeading().getRad(), 1e-9);
        }
    }
}
