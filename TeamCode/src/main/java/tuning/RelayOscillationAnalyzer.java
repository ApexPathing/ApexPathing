package tuning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Extracts repeatable oscillation amplitude and period from a bounded relay-feedback test based on
 * Åström and Hägglund's automatic-tuning method:
 * https://lup.lub.lu.se/record/8601786
 */
final class RelayOscillationAnalyzer {
    private static final int WINDOW_CYCLES = 5;
    private static final int MIN_CYCLES_BEFORE_ACCEPT = 6;
    private static final double MAX_AMPLITUDE_RELATIVE_SPREAD = 0.15;
    private static final double MAX_PERIOD_RELATIVE_SPREAD = 0.12;

    static final class Estimate {
        final double amplitude;
        final double periodSeconds;
        final double ultimateGain;
        final double amplitudeRelativeSpread;
        final double periodRelativeSpread;

        Estimate(double amplitude, double periodSeconds, double ultimateGain,
                 double amplitudeRelativeSpread, double periodRelativeSpread) {
            this.amplitude = amplitude;
            this.periodSeconds = periodSeconds;
            this.ultimateGain = ultimateGain;
            this.amplitudeRelativeSpread = amplitudeRelativeSpread;
            this.periodRelativeSpread = periodRelativeSpread;
        }
    }

    private final double relayPower;
    private final double hysteresis;
    private final List<Double> amplitudes = new ArrayList<Double>();
    private final List<Double> periods = new ArrayList<Double>();

    private int commandSign = 1;
    private boolean cycleStarted;
    private double cycleStartSeconds;
    private double cycleMinimum;
    private double cycleMaximum;

    RelayOscillationAnalyzer(double relayPower, double hysteresis) {
        if (!Double.isFinite(relayPower) || relayPower <= 0.0 || relayPower > 1.0 ||
                !Double.isFinite(hysteresis) || hysteresis <= 0.0) {
            throw new IllegalArgumentException(
                    "Relay power and hysteresis must be finite and positive");
        }
        this.relayPower = relayPower;
        this.hysteresis = hysteresis;
    }

    void observe(double elapsedSeconds, double position) {
        if (!Double.isFinite(elapsedSeconds) || !Double.isFinite(position)) { return; }

        if (cycleStarted) {
            cycleMinimum = Math.min(cycleMinimum, position);
            cycleMaximum = Math.max(cycleMaximum, position);
        }

        if (commandSign > 0 && position >= hysteresis) {
            commandSign = -1;
            if (cycleStarted) {
                double period = elapsedSeconds - cycleStartSeconds;
                double amplitude = (cycleMaximum - cycleMinimum) / 2.0;
                if (period > 0.0 && amplitude > hysteresis &&
                        Double.isFinite(period) && Double.isFinite(amplitude)) {
                    periods.add(period);
                    amplitudes.add(amplitude);
                }
            }
            cycleStarted = true;
            cycleStartSeconds = elapsedSeconds;
            cycleMinimum = position;
            cycleMaximum = position;
        } else if (commandSign < 0 && position <= -hysteresis) {
            commandSign = 1;
        }
    }

    double getCommand() { return commandSign * relayPower; }

    int getCycleCount() { return amplitudes.size(); }

    boolean hasStableEstimate() {
        if (amplitudes.size() < MIN_CYCLES_BEFORE_ACCEPT) { return false; }
        Estimate estimate = estimate();
        return estimate.amplitudeRelativeSpread <= MAX_AMPLITUDE_RELATIVE_SPREAD &&
                estimate.periodRelativeSpread <= MAX_PERIOD_RELATIVE_SPREAD;
    }

    Estimate estimate() {
        if (amplitudes.size() < WINDOW_CYCLES) {
            throw new IllegalStateException("At least five complete relay cycles are required");
        }
        double[] recentAmplitudes = tail(amplitudes, WINDOW_CYCLES);
        double[] recentPeriods = tail(periods, WINDOW_CYCLES);
        double amplitude = median(recentAmplitudes);
        double period = median(recentPeriods);
        double amplitudeRelativeSpread = relativeSpread(recentAmplitudes, amplitude);
        double periodRelativeSpread = relativeSpread(recentPeriods, period);
        double ultimateGain = 4.0 * relayPower / (Math.PI * amplitude);
        return new Estimate(amplitude, period, ultimateGain,
                amplitudeRelativeSpread, periodRelativeSpread);
    }

    private static double[] tail(List<Double> source, int count) {
        double[] result = new double[count];
        int offset = source.size() - count;
        for (int i = 0; i < count; i++) { result[i] = source.get(offset + i); }
        return result;
    }

    private static double relativeSpread(double[] values, double center) {
        double maximumDeviation = 0.0;
        for (double value : values) {
            maximumDeviation = Math.max(maximumDeviation, Math.abs(value - center));
        }
        return maximumDeviation / Math.max(Math.abs(center), 1e-9);
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int middle = sorted.length / 2;
        return sorted.length % 2 == 0
                ? (sorted[middle - 1] + sorted[middle]) / 2.0
                : sorted[middle];
    }
}
