package localizers.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.function.LongSupplier;

public class AdaptiveKalmanFilterTest {
    private static final double DT = 0.02;

    @Test
    public void tunedFilterRespondsFasterThanSevenSampleAverage() {
        FakeClock clock = new FakeClock();
        AdaptiveKalmanFilter kalman = new AdaptiveKalmanFilter(clock);
        kalman.setTuning(new AdaptiveKalmanFilter.KalmanTuning(0.04, 250.0));

        for (int i = 0; i < 20; i++) {
            update(kalman, clock, 0.0, DT);
        }

        double[] kalmanStep = new double[4];
        for (int i = 0; i < kalmanStep.length; i++) {
            kalmanStep[i] = update(kalman, clock, 1.0, DT).value();
        }

        // A seven-sample average reaches only 4/7 after four samples.
        assertTrue(kalmanStep[3] > 4.0 / 7.0);
        assertTrue(kalmanStep[0] > 0.0);
    }

    @Test
    public void followsRampsDirectionChangesAndVariableLoopPeriods() {
        FakeClock clock = new FakeClock();
        AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(clock);
        filter.setTuning(new AdaptiveKalmanFilter.KalmanTuning(0.02, 300.0));
        double estimate = 0.0;
        for (int i = 0; i < 80; i++) {
            double dt = i % 3 == 0 ? 0.01 : (i % 3 == 1 ? 0.02 : 0.035);
            estimate = update(filter, clock, i * 0.1, dt).value();
        }
        assertTrue(estimate > 6.0);

        for (int i = 0; i < 35; i++) {
            estimate = update(filter, clock, -4.0, 0.015 + (i % 2) * 0.02).value();
        }
        assertTrue(estimate < -3.0);
        assertFalse(filter.isTimeAnomalyDetected());
    }

    @Test
    public void rejectsSingleOutlierAndRecovers() {
        FakeClock clock = new FakeClock();
        AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(clock);
        filter.setTuning(new AdaptiveKalmanFilter.KalmanTuning(0.01, 30.0));
        for (int i = 0; i < 30; i++) {
            update(filter, clock, 2.0 + (i % 2 == 0 ? 0.02 : -0.02), DT);
        }
        double outlier = update(filter, clock, 100.0, DT).value();
        assertTrue(outlier < 20.0);
        double recovered = 0.0;
        for (int i = 0; i < 15; i++) {
            recovered = update(filter, clock, 2.0, DT).value();
        }
        assertEquals(2.0, recovered, 0.25);
    }

    @Test
    public void resetClearsStateButKeepsFrozenTuning() {
        FakeClock clock = new FakeClock();
        AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(clock);
        AdaptiveKalmanFilter.KalmanTuning tuning =
                new AdaptiveKalmanFilter.KalmanTuning(0.125, 42.0);
        filter.setTuning(tuning);
        update(filter, clock, 8.0, DT);
        filter.reset();

        assertEquals(0.125, filter.getTuning().measurementVariance, 0.0);
        assertEquals(42.0, filter.getTuning().processVariance, 0.0);
        assertEquals(-3.0, update(filter, clock, -3.0, DT).value(), 0.0);
    }

    @Test
    public void explicitCalibrationCanBeFrozen() {
        FakeClock clock = new FakeClock();
        AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(clock);
        filter.setAutoTuning(true);
        for (int i = 0; i < 100; i++) {
            update(filter, clock, 3.0 + Math.sin(i) * 0.05, DT);
        }
        filter.setAutoTuning(false);
        AdaptiveKalmanFilter.KalmanTuning frozen = filter.getTuning();
        for (int i = 0; i < 50; i++) {
            update(filter, clock, i % 2 == 0 ? 2.8 : 3.2, DT);
        }
        assertEquals(frozen.measurementVariance,
                filter.getTuning().measurementVariance, 0.0);
        assertEquals(frozen.processVariance, filter.getTuning().processVariance, 0.0);
    }

    @Test
    public void remainsStableAcrossNoiseLevelsAndLoopPeriods() {
        double[] noiseLevels = {0.01, 0.05, 0.20};
        double[] loopPeriods = {0.01, 0.02, 0.04};
        for (double noise : noiseLevels) {
            for (double dt : loopPeriods) {
                FakeClock clock = new FakeClock();
                AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(clock);
                filter.setTuning(new AdaptiveKalmanFilter.KalmanTuning(
                        noise * noise, noise * noise * 2500.0));
                double rawSquaredError = 0.0;
                double filteredSquaredError = 0.0;
                for (int i = 0; i < 240; i++) {
                    double deterministicNoise = noise * (Math.sin(i * 1.7)
                            + 0.45 * Math.sin(i * 0.37));
                    double estimate = update(filter, clock,
                            3.0 + deterministicNoise, dt).value();
                    if (i >= 40) {
                        rawSquaredError += deterministicNoise * deterministicNoise;
                        filteredSquaredError += (estimate - 3.0) * (estimate - 3.0);
                    }
                }
                assertTrue("Filter amplified steady-state noise at dt=" + dt
                                + ", noise=" + noise,
                        filteredSquaredError <= rawSquaredError * 1.25);
            }
        }
    }

    private static FilterState update(AdaptiveKalmanFilter filter, FakeClock clock,
                                      double measurement, double dt) {
        clock.advance(dt);
        return filter.update(measurement);
    }

    private static final class FakeClock implements LongSupplier {
        private long nanos;

        void advance(double seconds) {
            nanos += Math.round(seconds * 1.0e9);
        }

        @Override
        public long getAsLong() {
            return nanos;
        }
    }
}
