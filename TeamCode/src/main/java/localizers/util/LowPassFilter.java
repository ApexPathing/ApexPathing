package localizers.util;

/**
 * A simple low-pass filter to handle more accurate velocity and acceleration measurements.
 *
 * @author DrPixelCat - 7842 alum
 */
public class LowPassFilter implements DataFilter {
    private final int DEFAULT_SAMPLE_SIZE = 5;
    private double[] vals;
    private double[] rateVals;
    private boolean isWindowFull = false;
    private int size;
    private int iterator = 0;

    private double sum = 0;
    private double rateSum = 0;

    private long lastUpdateTimeNanos = -1;
    private double lastRawValue = 0;

    public LowPassFilter() {
        this.size = DEFAULT_SAMPLE_SIZE;
        vals = new double[this.size];
        rateVals = new double[this.size];
    }

    public void reset() {
        vals = new double[size];
        rateVals = new double[size];
        iterator = 0;
        sum = 0;
        rateSum = 0;
        isWindowFull = false;
        lastUpdateTimeNanos = -1;
        lastRawValue = 0;
    }

    public void setSampleSize(int size) {
        this.size = Math.max(1, size);
        reset();
    }

    public boolean isWindowFull() {
        return isWindowFull;
    }

    @Override
    public FilterState update(double measurement) {
        long currentNanos = System.nanoTime();
        double dt = 0.02; // Default starting assumption
        double rawRate = 0.0;

        // Calculate the raw rate of change using system time
        if (lastUpdateTimeNanos != -1) {
            dt = (currentNanos - lastUpdateTimeNanos) / 1.0e9;
            if (dt <= 1e-4) {
                dt = 1e-4; // Prevent division by zero on double-calls
            }
            rawRate = (measurement - lastRawValue) / dt;
        }

        lastUpdateTimeNanos = currentNanos;
        lastRawValue = measurement;

        // Advance the circular buffer
        iterator++;
        if (iterator > size - 1) {
            isWindowFull = true;
            iterator = 0;
        }

        // Update the value moving average
        sum -= vals[iterator];
        vals[iterator] = measurement;
        sum += measurement;

        // Update the rate moving average
        rateSum -= rateVals[iterator];
        rateVals[iterator] = rawRate;
        rateSum += rawRate;

        double smoothedValue;
        double smoothedRate;

        if (isWindowFull) {
            smoothedValue = sum / size;
            smoothedRate = rateSum / size;
        } else {
            smoothedValue = sum / iterator;
            smoothedRate = rateSum / iterator;
        }

        return new FilterState(smoothedValue, smoothedRate);
    }
}