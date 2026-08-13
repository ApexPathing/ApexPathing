package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class FeedforwardTunerTest {
    @Test
    public void robustRegressionRecoversFeedforwardFromAllFourRuns() {
        double kS = 0.08;
        double expectedKV = 0.025;
        double expectedKA = 0.004;
        List<FeedforwardTuner.Observation> samples = new ArrayList<>();

        for (int run = 0; run < 4; run++) {
            for (int i = 1; i <= 35; i++) {
                double velocity = 2.0 + i * 0.7 + run * 0.15;
                double acceleration = run < 2 ? 0.3 + i * 0.01 : 8.0 + i * 0.2;
                double noise = ((i % 5) - 2) * 0.0005;
                double power = kS + expectedKV * velocity + expectedKA * acceleration + noise;
                if (run == 2 && i == 17) { power += 0.35; }
                samples.add(new FeedforwardTuner.Observation(
                        power, velocity, acceleration, run));
            }
        }

        FeedforwardTuner.FitResult fit = FeedforwardTuner.fitFeedforward(samples, kS);

        assertTrue(fit.isValid());
        assertEquals(expectedKV, fit.kV, 0.001);
        assertEquals(expectedKA, fit.kA, 0.001);
        assertEquals(140, fit.sampleCount);
        assertTrue(fit.rSquared > 0.95);
    }

    @Test
    public void unrelatedVelocityAndAccelerationDoNotPassValidation() {
        double kS = 0.08;
        List<FeedforwardTuner.Observation> samples = new ArrayList<>();
        for (int run = 0; run < 4; run++) {
            for (int i = 1; i <= 30; i++) {
                double velocity = i * 0.5;
                double acceleration = ((i * 7 + run * 3) % 11) - 5.0;
                double power = 0.15 + ((i * 13 + run * 5) % 17) * 0.03;
                samples.add(new FeedforwardTuner.Observation(
                        power, velocity, acceleration, run));
            }
        }

        FeedforwardTuner.FitResult fit = FeedforwardTuner.fitFeedforward(samples, kS);

        assertTrue(!fit.isValid());
    }

}
