package tuning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Extracts repeatable oscillation amplitude and period from a bounded relay-feedback test based on
 * Astrom and Hagglund's automatic-tuning method:
 * https://lup.lub.lu.se/record/8601786
 */
final class RelayOscillationAnalyzer {
    private static final int WINDOW_PHASE_PAIRS = 5;
    private static final int CYCLES_PER_PHASE_PAIR = 2;
    private static final int MIN_CYCLES_BEFORE_ACCEPT =
            WINDOW_PHASE_PAIRS * CYCLES_PER_PHASE_PAIR;
    private static final int RECOVERY_CYCLES_AFTER_UNSTABLE_WINDOW =
            (WINDOW_PHASE_PAIRS - 1) * CYCLES_PER_PHASE_PAIR;
    private static final double DEADLINE_MARGIN_FRACTION = 0.25;
    private static final double MIN_DEADLINE_MARGIN_SECONDS = 0.50;
    private static final double MAX_AMPLITUDE_RELATIVE_CENTRAL_SPREAD = 0.15;
    private static final double MAX_PERIOD_RELATIVE_CENTRAL_SPREAD = 0.12;

    static final class Estimate {
        final double amplitude;
        final double periodSeconds;
        final double ultimateGain;
        final double amplitudeRelativeCentralSpread;
        final double periodRelativeCentralSpread;

        Estimate(double amplitude, double periodSeconds, double ultimateGain,
                 double amplitudeRelativeCentralSpread, double periodRelativeCentralSpread) {
            this.amplitude = amplitude;
            this.periodSeconds = periodSeconds;
            this.ultimateGain = ultimateGain;
            this.amplitudeRelativeCentralSpread = amplitudeRelativeCentralSpread;
            this.periodRelativeCentralSpread = periodRelativeCentralSpread;
        }
    }

    private final double relayPower;
    private final double hysteresis;
    private final List<Double> amplitudes = new ArrayList<Double>();
    private final List<Double> periods = new ArrayList<Double>();
    private final List<Double> phasePairPeriods = new ArrayList<Double>();

    private int commandSign = 1;
    private boolean cycleStarted;
    private double cycleStartSeconds;
    private double cycleMinimum;
    private double cycleMaximum;
    private Double pendingPairPeriod;
    private int discardedCycleCount;

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
                    // A nonlinear drivetrain can settle into a period-two relay response: one
                    // cycle is long and the next correspondingly short even though their combined
                    // phase and amplitude are repeatable. Non-overlapping two-cycle windows expose
                    // that stable aggregate without counting either cycle twice.
                    if (pendingPairPeriod == null) {
                        pendingPairPeriod = period;
                    } else {
                        phasePairPeriods.add((pendingPairPeriod + period) / 2.0);
                        pendingPairPeriod = null;
                    }
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

    /**
     * Drops the cycle currently spanning a missed or severely delayed control-loop sample.
     * Keeping it would attribute an unobserved interval to the drivetrain dynamics and corrupt
     * both the measured amplitude and period. The pending period-two pair is also cleared so two
     * non-adjacent cycles are never averaged together.
     */
    void discardCurrentCycle() {
        if (cycleStarted) { discardedCycleCount++; }
        cycleStarted = false;
        pendingPairPeriod = null;
    }

    int getCycleCount() { return amplitudes.size(); }

    int getRequiredCycleCount() { return MIN_CYCLES_BEFORE_ACCEPT; }

    int getDiscardedCycleCount() { return discardedCycleCount; }

    int getUsableCycleCount() {
        return phasePairPeriods.size() * CYCLES_PER_PHASE_PAIR +
                (pendingPairPeriod == null ? 0 : 1);
    }

    boolean canEstimate() { return phasePairPeriods.size() >= WINDOW_PHASE_PAIRS; }

    boolean hasStableEstimate() {
        if (!canEstimate()) { return false; }
        Estimate estimate = estimate();
        return estimate.amplitudeRelativeCentralSpread <=
                MAX_AMPLITUDE_RELATIVE_CENTRAL_SPREAD &&
                estimate.periodRelativeCentralSpread <= MAX_PERIOD_RELATIVE_CENTRAL_SPREAD;
    }

    Estimate estimate() {
        if (!canEstimate()) {
            throw new IllegalStateException(
                    "At least ten complete relay cycles are required");
        }
        // Keep amplitude validation cycle-by-cycle. Averaging adjacent amplitudes could hide a
        // genuinely unstable high/low alternation, unlike period where that alternation is the
        // phase pattern being measured intentionally.
        double[] recentAmplitudes = tail(amplitudes, WINDOW_PHASE_PAIRS);
        double[] recentPeriods = tail(phasePairPeriods, WINDOW_PHASE_PAIRS);
        double amplitude = median(recentAmplitudes);
        double period = median(recentPeriods);
        double amplitudeRelativeCentralSpread = relativeCentralSpread(
                recentAmplitudes, amplitude);
        double periodRelativeCentralSpread = relativeCentralSpread(recentPeriods, period);
        double ultimateGain = 4.0 * relayPower / (Math.PI * amplitude);
        return new Estimate(amplitude, period, ultimateGain,
                amplitudeRelativeCentralSpread, periodRelativeCentralSpread);
    }

    /**
     * Chooses an absolute timeout from the periods already measured. Once enough cycles exist but
     * their window is not yet repeatable, two more cycles plus scheduling margin are reserved. A
     * single anomalous simulator/odometry interval can therefore age out without allowing an
     * unbounded robot test.
     */
    double recommendedTimeoutSeconds(double minimumSeconds, double maximumSeconds) {
        // One complete period is already enough to prove that the initial fixed deadline is
        // unrealistic. Waiting for a second period before extending it can abort a slow relay
        // while that second period is still in progress.
        if (periods.isEmpty()) { return minimumSeconds; }

        double conservativePeriod = 0.0;
        int first = Math.max(0, periods.size() -
                WINDOW_PHASE_PAIRS * CYCLES_PER_PHASE_PAIR);
        for (int i = first; i < periods.size(); i++) {
            conservativePeriod = Math.max(conservativePeriod, periods.get(i));
        }
        int cyclesStillNeeded = Math.max(0, MIN_CYCLES_BEFORE_ACCEPT - periods.size());
        int recoveryCycles = periods.size() >= MIN_CYCLES_BEFORE_ACCEPT
                ? RECOVERY_CYCLES_AFTER_UNSTABLE_WINDOW
                : 1;
        double schedulingMargin = Math.max(
                MIN_DEADLINE_MARGIN_SECONDS,
                conservativePeriod * DEADLINE_MARGIN_FRACTION
        );
        double projectedDeadline = cycleStartSeconds +
                (cyclesStillNeeded + recoveryCycles) * conservativePeriod + schedulingMargin;
        return Math.min(maximumSeconds, Math.max(minimumSeconds, projectedDeadline));
    }

    private static double[] tail(List<Double> source, int count) {
        double[] result = new double[count];
        int offset = source.size() - count;
        for (int i = 0; i < count; i++) { result[i] = source.get(offset + i); }
        return result;
    }

    private static double relativeCentralSpread(double[] values, double center) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        double lowerQuartile = percentile(sorted, 0.25);
        double upperQuartile = percentile(sorted, 0.75);
        return (upperQuartile - lowerQuartile) / Math.max(Math.abs(center), 1e-9);
    }

    private static double percentile(double[] sorted, double fraction) {
        double index = fraction * (sorted.length - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);
        if (lower == upper) { return sorted[lower]; }
        double interpolation = index - lower;
        return sorted[lower] + (sorted[upper] - sorted[lower]) * interpolation;
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
