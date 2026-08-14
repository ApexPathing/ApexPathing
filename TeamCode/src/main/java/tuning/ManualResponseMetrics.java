package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import drivetrains.BaseDrivetrain;

/** Compact scalar-response recorder shared by the manual position-controller tests. */
final class ManualResponseMetrics {
    private static final double SETTLE_HOLD_SECONDS = 0.25;

    private final ElapsedTime timer = new ElapsedTime();
    private TuningCsvWriter csv;
    private String csvPath = "Not started";
    private boolean active;
    private double start;
    private double target;
    private double positionTolerance;
    private double velocityTolerance;
    private double lastTime;
    private double squaredErrorIntegral;
    private double timeWeightedSquaredError;
    private double peakVelocity;
    private double overshoot;
    private double finalError = Double.NaN;
    private double settledSince = -1.0;
    private double settlingTime = Double.NaN;
    private int samples;
    private int saturatedSamples;

    void begin(String prefix, double start, double target,
               double positionTolerance, double velocityTolerance) {
        close();
        csv = TuningCsvWriter.open(prefix,
                "time_s", "target", "position", "error", "velocity", "max_motor_power");
        csvPath = csv.getPath();
        this.start = start;
        this.target = target;
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
        lastTime = 0.0;
        squaredErrorIntegral = 0.0;
        timeWeightedSquaredError = 0.0;
        peakVelocity = 0.0;
        overshoot = 0.0;
        finalError = target - start;
        settledSince = -1.0;
        settlingTime = Double.NaN;
        samples = 0;
        saturatedSamples = 0;
        active = true;
        timer.reset();
    }

    void sample(double position, double velocity, double maxMotorPower) {
        if (!active || !Double.isFinite(position) || !Double.isFinite(velocity)) { return; }
        double elapsed = timer.seconds();
        double dt = Math.max(0.0, elapsed - lastTime);
        double error = target - position;
        squaredErrorIntegral += error * error * dt;
        timeWeightedSquaredError += elapsed * error * error * dt;
        peakVelocity = Math.max(peakVelocity, Math.abs(velocity));
        double direction = Math.signum(target - start);
        overshoot = Math.max(overshoot, Math.max(0.0, direction * (position - target)));
        finalError = error;
        samples++;
        if (Double.isFinite(maxMotorPower) && maxMotorPower >= 0.99) { saturatedSamples++; }

        boolean withinTolerance = Math.abs(error) <= positionTolerance &&
                Math.abs(velocity) <= velocityTolerance;
        if (withinTolerance) {
            if (settledSince < 0.0) { settledSince = elapsed; }
            if (!Double.isFinite(settlingTime) &&
                    elapsed - settledSince >= SETTLE_HOLD_SECONDS) {
                settlingTime = elapsed;
            }
        } else {
            settledSince = -1.0;
        }
        if (csv != null) {
            csv.writeRow(elapsed, target, position, error, velocity, maxMotorPower);
        }
        lastTime = elapsed;
    }

    void finish() {
        active = false;
        close();
    }

    void close() {
        if (csv != null) {
            csv.close();
            csv = null;
        }
    }

    boolean isActive() { return active; }
    double getFinalError() { return finalError; }
    double getOvershoot() { return overshoot; }
    double getSettlingTime() { return settlingTime; }
    double getRmsError() {
        return lastTime > 0.0 ? Math.sqrt(squaredErrorIntegral / lastTime) : Double.NaN;
    }
    double getTimeWeightedSquaredError() { return timeWeightedSquaredError; }
    double getPeakVelocity() { return peakVelocity; }
    double getSaturationFraction() {
        return samples == 0 ? 0.0 : (double) saturatedSamples / samples;
    }
    String getCsvPath() { return csvPath; }

    static double maxMotorPower(BaseDrivetrain<?> drivetrain) {
        return Math.max(Math.max(Math.abs(drivetrain.getLastFlPower()),
                        Math.abs(drivetrain.getLastFrPower())),
                Math.max(Math.abs(drivetrain.getLastBlPower()),
                        Math.abs(drivetrain.getLastBrPower())));
    }
}
