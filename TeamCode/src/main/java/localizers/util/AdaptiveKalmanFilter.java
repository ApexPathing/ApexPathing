package localizers.util;

import geometry.Matrix;

import java.util.function.LongSupplier;

/**
 * A two-state linear Kalman filter designed for robotic kinematics (e.g., tracking [Velocity, Acceleration]).
 *
 * Unlike standard Kalman filters that require static, manually tuned Q and R covariance matrices,
 * this filter uses an innovation-based adaptive estimator. It watches the statistical spread of the
 * incoming data to learn the actual sensor noise (R) and monitors its own prediction errors (NIS)
 * to dynamically scale the process trust (Q).
 *
 * All matrix operations run through the lightweight {@link Matrix} class.
 *
 * @author Joel H - 7842a
 */
public class AdaptiveKalmanFilter implements DataFilter {
    private static final double MINIMUM_DT = 1e-4; // Near-zero threshold
    private static final double MAXIMUM_DT = 0.2;  // 200ms threshold

    private static final double MINIMUM_VARIANCE = 1e-24;
    private static final int BOOTSTRAP_SAMPLES = 10;
    private static final Matrix H = new Matrix(new double[][]{{1.0, 0.0}});
    private static final Matrix I = Matrix.identity(2);

    private boolean initialized;
    private final LongSupplier nanoTimeSource;
    private Matrix x; // State Vector [value, rate]^T
    private Matrix P; // Covariance Matrix
    private long lastUpdateTimeNanos = -1;
    private double lastDt = 0.02; // Default starting assumption (50Hz)
    private boolean timeAnomalyDetected = false;
    private double measurementVariance = 1.0;
    private double processVariance = 1.0;

    private double referenceDt = 0.02;
    private double previousMeasurement;
    private double previousInnovation;
    private double previousInnovationVariance = 1.0;
    private double innovationBias;
    private double smoothedNis = 1.0;
    private double smoothedInnovationCorrelation;

    private double adaptationRate = 0.02;
    private double outlierSigma = 4.5;
    private boolean isAutoTuning = false;

    private int bootstrapSamples;
    private int sameDirectionGateCount;
    private int previousGateDirection;
    private boolean previousMeasurementWasUsable;

    /**
     * A container for extracting and injecting tuned variance parameters.
     */
    public static class KalmanTuning {
        public final double measurementVariance;
        public final double processVariance;

        public KalmanTuning(double measurementVariance, double processVariance) {
            this.measurementVariance = measurementVariance;
            this.processVariance = processVariance;
        }
    }

    public AdaptiveKalmanFilter() {
        this(System::nanoTime);
    }

    /** Constructor exposed for deterministic estimator tests. */
    public AdaptiveKalmanFilter(LongSupplier nanoTimeSource) {
        if (nanoTimeSource == null) {
            throw new IllegalArgumentException("nanoTimeSource cannot be null");
        }
        this.nanoTimeSource = nanoTimeSource;
    }

    /**
     * Pushes the filter forward in time and assimilates a new raw sensor reading.
     *
     * This executes the full predict/update Kalman loop. If the auto-tuner is enabled,
     * it will also mutate the internal variance matrices based on the innovation error
     * of this specific measurement.
     *
     * @param measurement The raw, noisy sensor data (e.g., raw encoder velocity)
     * @return The smoothed state estimate.
     */
    @Override
    public FilterState update(double measurement) {
        long currentNanos = nanoTimeSource.getAsLong();
        double dt;

        if (lastUpdateTimeNanos == -1) {
            dt = lastDt;
        } else {
            dt = (currentNanos - lastUpdateTimeNanos) / 1.0e9;
        }
        lastUpdateTimeNanos = currentNanos;

        if (!Double.isFinite(measurement)) {
            return new FilterState(x != null ? x.get(0, 0) : 0, x != null ? x.get(1, 0) : 0);
        }

        // Time Anomaly Protection
        if (dt > MAXIMUM_DT || dt < MINIMUM_DT) {
            timeAnomalyDetected = true;
            dt = lastDt;
        } else {
            timeAnomalyDetected = false;
            lastDt = dt;
        }

        if (!initialized) {
            initialized = true;

            x = new Matrix(new double[][]{
                    {measurement},
                    {0.0}
            });

            referenceDt = dt;
            previousMeasurement = measurement;
            previousMeasurementWasUsable = true;
            bootstrapSamples = 0;
            sameDirectionGateCount = 0;
            previousGateDirection = 0;
            innovationBias = 0.0;
            smoothedNis = 1.0;
            smoothedInnovationCorrelation = 0.0;

            if (isAutoTuning) {
                measurementVariance = numericalVarianceFloor(measurement);
                processVariance = measurementVariance / Math.max(MINIMUM_VARIANCE, dt * dt * dt) * 0.001;
            }

            P = new Matrix(new double[][]{
                    {positive(measurementVariance * 4.0), 0.0},
                    {0.0, positive(measurementVariance / (dt * dt))}
            });

            return new FilterState(x.get(0, 0), x.get(1, 0));
        }

        referenceDt += elapsedAlpha(dt, 2.0) * (dt - referenceDt);

        // (x = Fx, P = FPF^T + Q)
        Matrix F = new Matrix(new double[][]{
                {1.0, dt},
                {0.0, 1.0}
        });

        double dt2 = dt * dt;
        Matrix Q = new Matrix(new double[][]{
                {dt2 * dt / 3.0, dt2 / 2.0},
                {dt2 / 2.0, dt}
        }).multiply(processVariance);

        Matrix xPred = F.multiply(x);
        Matrix PPred = F.multiply(P).multiply(F.transpose()).add(Q);

        double predictedRate = xPred.get(1, 0);

        // (y = z - Hx)
        Matrix z = new Matrix(new double[][]{{measurement}});

        Matrix y = z.subtract(H.multiply(xPred));
        double innovation = y.get(0, 0);

        Matrix R_base = new Matrix(new double[][]{{measurementVariance}});
        Matrix S_base = H.multiply(PPred).multiply(H.transpose()).add(R_base);
        double innovationVariance = positive(S_base.get(0, 0));

        double normalizedInnovation = innovation / Math.sqrt(innovationVariance);
        double nis = normalizedInnovation * normalizedInnovation;
        double gateSquared = outlierSigma * outlierSigma;

        boolean insideGate = nis <= gateSquared;
        updateGatePersistence(innovation, insideGate);

        if (isAutoTuning) {
            // Actively analyze the innovation and mutate the variances
            adaptNoiseEstimates(measurement, predictedRate, innovation, innovationVariance, nis, insideGate, dt);
        }

        // (x = x + Ky, P = Joseph Form)
        double robustScale = 1.0;
        if (!insideGate) {
            robustScale = Math.abs(normalizedInnovation) / outlierSigma;
            if (sameDirectionGateCount >= 3) {
                robustScale /= Math.min(4.0, 1.0 + sameDirectionGateCount);
            }
            robustScale = Math.max(1.0, robustScale);
        }

        Matrix R_eff = new Matrix(new double[][]{{measurementVariance * robustScale}});
        Matrix S = H.multiply(PPred).multiply(H.transpose()).add(R_eff);
        S = new Matrix(new double[][]{{positive(S.get(0, 0))}}); // Guarantee positive definiteness

        // Kalman Gain: K = P * H^T * S^-1
        Matrix K = PPred.multiply(H.transpose()).multiply(S.inverse());

        // Update State
        x = xPred.add(K.multiply(y));

        // Joseph Form Update: P = (I - KH) * P * (I - KH)^T + K * R_eff * K^T
        Matrix temp = I.subtract(K.multiply(H));
        P = temp.multiply(PPred).multiply(temp.transpose())
                .add(K.multiply(R_eff).multiply(K.transpose()));

        // Stabilize Covariance to prevent floating point drift
        double p00 = positive(P.get(0, 0));
        double p11 = positive(P.get(1, 1));
        double limit = 0.999999 * Math.sqrt(p00 * p11);
        double p01 = clamp(P.get(0, 1), -limit, limit);
        double p10 = clamp(P.get(1, 0), -limit, limit);
        double avgCov = (p01 + p10) / 2.0; // Force symmetry

        P = new Matrix(new double[][]{
                {p00, avgCov},
                {avgCov, p11}
        });

        previousMeasurement = measurement;
        previousMeasurementWasUsable = insideGate;
        previousInnovation = innovation;
        previousInnovationVariance = innovationVariance;

        return new FilterState(x.get(0, 0), x.get(1, 0));
    }

    private void adaptNoiseEstimates(
            double measurement,
            double predictedRate,
            double innovation,
            double innovationVariance,
            double nis,
            boolean insideGate,
            double dt
    ) {
        double baseRate = Math.max(0.001, adaptationRate);
        double measurementAlpha = elapsedAlpha(dt, 1.0 / (25.0 * baseRate));
        double processAlpha = elapsedAlpha(dt, 1.0 / (50.0 * baseRate));
        double biasAlpha = elapsedAlpha(dt, 0.25);

        if (bootstrapSamples == 0 || (insideGate && previousMeasurementWasUsable)) {
            double detrendedDifference = measurement - previousMeasurement - predictedRate * dt;
            double candidate = 0.5 * detrendedDifference * detrendedDifference;
            candidate = Math.max(candidate, numericalVarianceFloor(measurement));

            if (bootstrapSamples == 0 && candidate > numericalVarianceFloor(measurement)) {
                measurementVariance = candidate;
                bootstrapSamples = 1;
                rebaseCovarianceAfterBootstrap(dt);
            } else if (bootstrapSamples > 0) {
                double alpha = bootstrapSamples < BOOTSTRAP_SAMPLES
                        ? Math.max(measurementAlpha, 1.0 / (bootstrapSamples + 1.0))
                        : measurementAlpha;
                double winsorized = clamp(candidate, measurementVariance * 0.025, measurementVariance * 25.0);
                measurementVariance += alpha * (winsorized - measurementVariance);
                bootstrapSamples++;
            }
        }
        measurementVariance = positive(measurementVariance);

        if (insideGate) {
            double cappedNis = Math.min(nis, 9.0);
            smoothedNis += processAlpha * (cappedNis - smoothedNis);

            double correlation = previousMeasurementWasUsable
                    ? innovation * previousInnovation / Math.sqrt(positive(innovationVariance * previousInnovationVariance))
                    : 0.0;
            correlation = clamp(correlation, -2.0, 2.0);
            smoothedInnovationCorrelation += processAlpha * (correlation - smoothedInnovationCorrelation);

            double boundedInnovation = clamp(innovation,
                    -outlierSigma * Math.sqrt(innovationVariance),
                    outlierSigma * Math.sqrt(innovationVariance));
            innovationBias += biasAlpha * (boundedInnovation - innovationBias);
        } else {
            double boundedInnovation = Math.copySign(outlierSigma * Math.sqrt(innovationVariance), innovation);
            innovationBias += biasAlpha * (boundedInnovation - innovationBias);
        }

        double biasNis = innovationBias * innovationBias / innovationVariance;
        double expectedBiasNis = biasAlpha / Math.max(1e-6, 2.0 - biasAlpha);
        double drive = 0.45 * (smoothedNis - 1.0)
                + 1.8 * (biasNis - expectedBiasNis)
                + 0.35 * Math.max(0.0, smoothedInnovationCorrelation);

        drive = clamp(drive, -0.40, 3.0);
        processVariance *= Math.exp(processAlpha * drive);

        if (!insideGate && sameDirectionGateCount >= 3) {
            processVariance *= Math.exp(processAlpha * Math.min(8.0, sameDirectionGateCount));
            double maneuverScale = innovation * innovation / Math.max(MINIMUM_VARIANCE, dt * dt * dt);
            processVariance = Math.max(processVariance, maneuverScale * 1e-3);
        }

        boundAutomaticProcessNoise();
    }

    private void updateGatePersistence(double innovation, boolean insideGate) {
        if (insideGate || innovation == 0.0) {
            sameDirectionGateCount = 0;
            previousGateDirection = 0;
            return;
        }
        int direction = innovation > 0.0 ? 1 : -1;
        if (direction == previousGateDirection) {
            sameDirectionGateCount++;
        } else {
            sameDirectionGateCount = 1;
            previousGateDirection = direction;
        }
    }

    private void rebaseCovarianceAfterBootstrap(double dt) {
        processVariance = measurementVariance
                / Math.max(MINIMUM_VARIANCE, dt * dt * dt)
                * 0.001;

        double currentP00 = Math.max(P.get(0, 0), measurementVariance * 2.0);
        double currentP11 = Math.max(P.get(1, 1), measurementVariance / (dt * dt));

        P = new Matrix(new double[][]{
                {currentP00, P.get(0, 1)},
                {P.get(1, 0), currentP11}
        });
    }

    private void boundAutomaticProcessNoise() {
        double dt3 = Math.max(MINIMUM_VARIANCE, referenceDt * referenceDt * referenceDt);
        double scale = measurementVariance / dt3;

        processVariance = clamp(
                processVariance,
                Math.max(MINIMUM_VARIANCE, scale * 1e-3),
                Math.max(MINIMUM_VARIANCE, scale * 1e3)
        );
    }

    private double elapsedAlpha(double dt, double timeConstant) {
        return -Math.expm1(-dt / Math.max(MINIMUM_DT, timeConstant));
    }

    private double numericalVarianceFloor(double val) {
        double ulp = Math.ulp(Math.max(1.0, Math.abs(val)));
        return Math.max(MINIMUM_VARIANCE, 256.0 * ulp * ulp);
    }

    private double positive(double val) {
        return Double.isFinite(val) ? Math.max(MINIMUM_VARIANCE, val) : 1.0;
    }

    private double clamp(double val, double low, double high) {
        return Math.max(low, Math.min(high, val));
    }

    public KalmanTuning getTuning() {
        return new KalmanTuning(measurementVariance, processVariance);
    }

    public void setTuning(KalmanTuning tuning) {
        this.measurementVariance = Math.max(MINIMUM_VARIANCE, tuning.measurementVariance);
        this.processVariance = Math.max(MINIMUM_VARIANCE, tuning.processVariance);
    }

    /**
     * Toggles the innovation-based learning phase.
     *
     * When enabled, the filter actively modifies its process and measurement variances
     * to lock onto the current sensor noise profile.
     */
    public void setAutoTuning(boolean enabled) {
        if (this.isAutoTuning == enabled) {
            return;
        }
        this.isAutoTuning = enabled;
        if (enabled) {
            smoothedNis = 1.0;
            smoothedInnovationCorrelation = 0.0;
            bootstrapSamples = 0;
            if (initialized) {
                measurementVariance = x != null ? numericalVarianceFloor(x.get(0, 0)) : MINIMUM_VARIANCE;
                processVariance = measurementVariance / Math.max(MINIMUM_VARIANCE, referenceDt * referenceDt * referenceDt) * 0.001;
            }
        }
    }

    /**
     * Determines if the filter has finished hunting for the baseline sensor noise.
     *
     * @return true if the initial bootstrap phase is complete and the Normalized
     *         Innovation Squared (NIS) has settled into a mathematically stable pocket.
     */
    public boolean isTuned() {
        if (!isAutoTuning) {
            return true;
        }
        boolean bootstrapComplete = bootstrapSamples >= BOOTSTRAP_SAMPLES;
        boolean nisStable = smoothedNis > 0.9 && smoothedNis < 1.1;
        return initialized && bootstrapComplete && nisStable;
    }

    /**
     * Configures the sensitivity of the auto-tuner.
     */
    public void setTuningParameters(double adaptationRate, double outlierSigma) {
        this.adaptationRate = clamp(adaptationRate, 0.001, 0.20);
        this.outlierSigma = clamp(outlierSigma, 2.0, 10.0);
    }

    /**
     * Checks if the loop time violated the safety bounds during the last update.
     *
     * @return true if the last dt was overridden to protect the math.
     */
    public boolean isTimeAnomalyDetected() {
        return timeAnomalyDetected;
    }

    /**
     * Wipes the entire state and variance tracking cleanly back to zero.
     */
    @Override
    public void reset() {
        initialized = false;

        x = new Matrix(new double[][]{{0.0}, {0.0}});
        P = new Matrix(new double[][]{{0.0, 0.0}, {0.0, 0.0}});

        if (isAutoTuning) {
            measurementVariance = MINIMUM_VARIANCE;
            processVariance = MINIMUM_VARIANCE;
        }

        referenceDt = 0.02;
        lastDt = 0.02;
        lastUpdateTimeNanos = -1;
        timeAnomalyDetected = false;

        previousMeasurement = previousInnovation = 0.0;
        previousInnovationVariance = 1.0;
        innovationBias = 0.0;
        smoothedNis = 1.0;
        smoothedInnovationCorrelation = 0.0;
        bootstrapSamples = 0;
        sameDirectionGateCount = 0;
        previousGateDirection = 0;
        previousMeasurementWasUsable = false;
    }
}
