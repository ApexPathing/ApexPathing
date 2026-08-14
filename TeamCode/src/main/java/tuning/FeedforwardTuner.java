package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import geometry.AngleUnit;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import localizers.util.LowPassFilter;
import localizers.BaseLocalizer;

/**
 * Tunes the feedforward coefficients (kV and kA) for both the angular (heading) and
 * translational (drive) controllers. Automatic tuning characterizes forward and reverse motion
 * with quasistatic and dynamic tests, then robustly fits and cross-validates both coefficients.
 *
 * @author Joel H - 7842a
 */
public class FeedforwardTuner extends TuningPhase {
    private static final double QUASISTATIC_TIME = 3.0;
    private static final double DYNAMIC_TIME = 1.25;
    private static final double CHARACTERIZATION_POWER = 0.70;
    private static final double AUTO_SETTLE_TIME = 0.40;
    private static final double MIN_SAMPLE_TIME = 0.12;
    private static final double STATIONARY_LINEAR_SPEED_IN_PER_SEC = 1.0;
    private static final double STATIONARY_ANGULAR_SPEED_RAD_PER_SEC = 0.10;
    private static final double MAX_CROSS_VALIDATION_RMSE = 0.15;
    // A drivetrain includes static-friction transitions that a two-term linear fit cannot
    // explain perfectly. Held-out-run RMSE remains the primary repeatability safeguard.
    private static final double MIN_R_SQUARED = 0.65;

    private enum Coefficient { ANGULAR_KV, ANGULAR_KA, TRANSLATIONAL_KV, TRANSLATIONAL_KA }

    private enum ManualState { IDLE, ANGULAR, DRIVE }
    private ManualState manualState = ManualState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private TrapezoidProfile driveProfile;
    private TrapezoidProfile angularProfile;

    private Coefficient selected = Coefficient.ANGULAR_KV;

    private enum Axis { ANGULAR, TRANSLATIONAL }
    private enum Excitation { QUASISTATIC, DYNAMIC }
    private enum AutoStage { PROMPT, RUNNING, SETTLING, FITTING, FAILED, DONE }

    private static final class AutoRun {
        final Axis axis;
        final Excitation excitation;
        final boolean forward;

        AutoRun(Axis axis, Excitation excitation, boolean forward) {
            this.axis = axis;
            this.excitation = excitation;
            this.forward = forward;
        }
    }

    private static final AutoRun[] AUTO_RUNS = {
            new AutoRun(Axis.ANGULAR, Excitation.QUASISTATIC, true),
            new AutoRun(Axis.ANGULAR, Excitation.QUASISTATIC, false),
            new AutoRun(Axis.ANGULAR, Excitation.DYNAMIC, true),
            new AutoRun(Axis.ANGULAR, Excitation.DYNAMIC, false),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.QUASISTATIC, true),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.QUASISTATIC, false),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.DYNAMIC, true),
            new AutoRun(Axis.TRANSLATIONAL, Excitation.DYNAMIC, false)
    };

    static final class Observation {
        final double power;
        final double velocity;
        final double acceleration;
        final int run;
        final double elapsedSeconds;

        Observation(double power, double velocity, double acceleration, int run) {
            this(power, velocity, acceleration, run, Double.NaN);
        }

        Observation(double power, double velocity, double acceleration, int run,
                    double elapsedSeconds) {
            this.power = power;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.run = run;
            this.elapsedSeconds = elapsedSeconds;
        }
    }

    static final class FitResult {
        final double kV;
        final double kA;
        final double rmse;
        final double crossValidationRmse;
        final double rSquared;
        final int sampleCount;

        FitResult(double kV, double kA, double rmse, double crossValidationRmse,
                  double rSquared,
                  int sampleCount) {
            this.kV = kV;
            this.kA = kA;
            this.rmse = rmse;
            this.crossValidationRmse = crossValidationRmse;
            this.rSquared = rSquared;
            this.sampleCount = sampleCount;
        }

        boolean isValid() {
            return Double.isFinite(kV) && Double.isFinite(kA) &&
                    kV > 0.0 && kA > 0.0 && sampleCount >= 20 &&
                    Double.isFinite(crossValidationRmse) &&
                    crossValidationRmse <= MAX_CROSS_VALIDATION_RMSE &&
                    Double.isFinite(rSquared) && rSquared >= MIN_R_SQUARED;
        }
    }

    private final List<Observation> angularObservations = new ArrayList<>();
    private final List<Observation> translationalObservations = new ArrayList<>();
    private AutoStage autoStage = AutoStage.PROMPT;
    private int autoRunIndex = 0;
    private FitResult angularFit;
    private FitResult translationalFit;
    private String validationMessage = "Not run";
    private String csvPath = "Not written";
    private String csvError;
    private boolean isForward = true;
    private boolean manualDriveHasRun = false;
    private boolean manualAngularPositive = true;
    private boolean manualAngularHasRun = false;
    private int manualRunNumber;
    private int manualSamples;
    private int manualSaturatedSamples;
    private double manualErrorSquared;
    private double manualPeakError;
    private double manualPeakVelocity;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private final LowPassFilter commandPowerFilter = new LowPassFilter();
    private double lastAppliedCharacterizationPower;

    public FeedforwardTuner(TunerContext context) {
        super(context);
    }

    @Override
    protected String getPhaseName() {
        return "Feedforward Refinement";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Manual angular runs alternate counterclockwise and clockwise.");
        context.getTelemetry().addLine(
                "Manual drive runs need at least 72 inches clear in the selected direction.");
    }

    @Override
    protected void init() {
        // Target half of the physical limits to avoid motor saturation during tuning.
        angularProfile = new TrapezoidProfile(
                context.constants.angularVelLimitRad / 2,
                context.constants.angularAccelLimitRad / 2
        );
        driveProfile = new TrapezoidProfile(
                context.constants.forwardVelLimitIn / 2,
                context.constants.forwardAccelLimitIn / 2
        );

        timer.reset();
        isForward = true;
        manualDriveHasRun = false;
        manualAngularPositive = true;
        manualAngularHasRun = false;
        if (manualMode) {
            manualState = ManualState.IDLE;
            context.getFollower().stop();
            manualRunNumber = 0;
            manualCsv = TuningCsvWriter.open("manual_feedforward_response",
                    "run", "time_s", "axis", "direction", "target_velocity",
                    "actual_velocity", "error", "raw_power", "applied_power", "saturated");
            manualCsvPath = manualCsv.getPath();
        } else {
            restartAutomaticCharacterization();
        }
    }

    private void restartAutomaticCharacterization() {
        context.getFollower().stop();
        angularObservations.clear();
        translationalObservations.clear();
        angularFit = null;
        translationalFit = null;
        validationMessage = "Collecting characterization data";
        csvPath = "Pending";
        csvError = null;
        autoRunIndex = 0;
        autoStage = AutoStage.PROMPT;
        if (usesMovingAverageVelocity()) {
            commandPowerFilter.setSampleSize(
                    context.getFollower().getLocalizer().getFilterWindowSize());
        } else {
            commandPowerFilter.setSampleSize(1);
        }
        lastAppliedCharacterizationPower = 0.0;
        timer.reset();
    }


    /**
     * Generates a 1-second cruise trapezoidal motion profile to evaluate feedforward consistency.
     */
    public static class TrapezoidProfile {
        private final double accel;
        private final double vel;
        private final double tAccelEnd;
        private final double tCruiseEnd;
        private final double tEnd;

        public TrapezoidProfile(double vel, double accel) {
            this.vel = Math.abs(vel);
            this.accel = Math.abs(accel);

            if (!Double.isFinite(this.vel) || !Double.isFinite(this.accel) ||
                    this.vel <= 0.0 || this.accel <= 0.0) {
                throw new IllegalArgumentException(
                        "Trapezoid profile velocity and acceleration must be finite and positive."
                );
            }

            // Phase 1: Ramping up (v = at -> t = v/a)
            this.tAccelEnd = this.vel / this.accel;
            // Phase 2: Cruising for exactly 1.0 second
            this.tCruiseEnd = this.tAccelEnd + 1.0;
            // Phase 3: Ramping down (takes the same time as ramping up)
            this.tEnd = this.tCruiseEnd + this.tAccelEnd;
        }

        public double getAccel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel;
            }
            if (t < tCruiseEnd) {
                return 0.0;
            }
            return -accel;
        }

        public double getVel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel * t;
            }
            if (t < tCruiseEnd) {
                return vel;
            }
            // Deceleration: Max velocity minus what has been lost over time
            return vel - accel * (t - tCruiseEnd);
        }

        public double getTotalTime() {
            return tEnd;
        }

        public double getAccelEnd() {
            return tAccelEnd;
        }

        public double getCruiseEnd() {
            return tCruiseEnd;
        }

        public double getTotalDistance() {
            return vel * tCruiseEnd;
        }
    }

    /**
     * Fits normalized motor power = kS + kV * |velocity| + kA * directed acceleration.
     *
     * <p>All samples participate instead of reducing a run to one boundary decision. Four Huber
     * reweighting passes prevent an encoder/localizer spike from dominating the coefficients.</p>
     */
    static FitResult fitFeedforward(List<Observation> source, double kS) {
        List<Observation> samples = new ArrayList<>();
        for (Observation sample : source) {
            if (Double.isFinite(sample.power) && Double.isFinite(sample.velocity) &&
                    Double.isFinite(sample.acceleration) && sample.power > kS &&
                    sample.velocity > 0.0) {
                samples.add(sample);
            }
        }
        if (samples.size() < 4) {
            return new FitResult(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    samples.size());
        }

        double[] fit = robustFit(samples, kS, -1);
        double squaredError = 0.0;
        double totalSquaredError = 0.0;
        for (Observation sample : samples) {
            double residual = predictionError(sample, kS, fit);
            squaredError += residual * residual;
            double target = sample.power - kS;
            totalSquaredError += target * target;
        }
        double rmse = Math.sqrt(squaredError / samples.size());
        // kS is a known intercept, so the remaining kV/kA model is constrained through zero.
        // Use the corresponding uncentered R-squared; centering around the narrow sampled-power
        // mean can report a negative score for an otherwise accurate physical fit.
        double rSquared = totalSquaredError <= 1e-12
                ? Double.NaN : 1.0 - squaredError / totalSquaredError;

        // Leave each physical run out once. This detects a fit that only works in one direction
        // or for one excitation profile without requiring another long robot movement.
        double cvSquaredError = 0.0;
        int cvCount = 0;
        for (int heldOut = 0; heldOut < 4; heldOut++) {
            double[] foldFit = robustFit(samples, kS, heldOut);
            if (!Double.isFinite(foldFit[0]) || !Double.isFinite(foldFit[1])) { continue; }
            for (Observation sample : samples) {
                if (sample.run == heldOut) {
                    double residual = predictionError(sample, kS, foldFit);
                    cvSquaredError += residual * residual;
                    cvCount++;
                }
            }
        }
        double cvRmse = cvCount == 0 ? Double.NaN : Math.sqrt(cvSquaredError / cvCount);
        return new FitResult(fit[0], fit[1], rmse, cvRmse, rSquared, samples.size());
    }

    private static double predictionError(Observation sample, double kS, double[] fit) {
        return (sample.power - kS) -
                (fit[0] * sample.velocity + fit[1] * sample.acceleration);
    }

    private static double[] robustFit(List<Observation> samples, double kS, int excludedRun) {
        double[] weights = new double[samples.size()];
        Arrays.fill(weights, 1.0);
        double[] result = { Double.NaN, Double.NaN };

        for (int iteration = 0; iteration < 4; iteration++) {
            double vv = 1e-9;
            double va = 0.0;
            double aa = 1e-9;
            double vy = 0.0;
            double ay = 0.0;
            int used = 0;
            for (int i = 0; i < samples.size(); i++) {
                Observation sample = samples.get(i);
                if (sample.run == excludedRun) { continue; }
                double weight = weights[i];
                double target = sample.power - kS;
                vv += weight * sample.velocity * sample.velocity;
                va += weight * sample.velocity * sample.acceleration;
                aa += weight * sample.acceleration * sample.acceleration;
                vy += weight * sample.velocity * target;
                ay += weight * sample.acceleration * target;
                used++;
            }
            double determinant = vv * aa - va * va;
            if (used < 4 || Math.abs(determinant) < 1e-12) { return result; }
            result[0] = (vy * aa - ay * va) / determinant;
            result[1] = (ay * vv - vy * va) / determinant;

            double[] residuals = new double[samples.size()];
            int residualCount = 0;
            for (Observation sample : samples) {
                if (sample.run != excludedRun) {
                    residuals[residualCount++] = Math.abs(predictionError(sample, kS, result));
                }
            }
            double scale = LimitsPhase.percentile(
                    Arrays.copyOf(residuals, residualCount), 0.50) * 1.4826;
            scale = Math.max(scale, 0.005);
            double huberLimit = 1.5 * scale;
            for (int i = 0; i < samples.size(); i++) {
                Observation sample = samples.get(i);
                double residual = Math.abs(predictionError(sample, kS, result));
                weights[i] = residual <= huberLimit ? 1.0 : huberLimit / residual;
            }
        }
        return result;
    }

    @Override
    protected boolean manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = selected == Coefficient.ANGULAR_KV ? Coefficient.TRANSLATIONAL_KA :
                    Coefficient.values()[selected.ordinal() - 1];
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = selected == Coefficient.TRANSLATIONAL_KA ? Coefficient.ANGULAR_KV :
                    Coefficient.values()[selected.ordinal() + 1];
        }

        double change = manualChange();
        boolean coefficientsChanged = change != 0.0;
        if (change != 0.0) {
            if (selected == Coefficient.ANGULAR_KV) {
                context.constants.angularKV = Math.max(
                        0.0, context.constants.angularKV + change
                );
            } else if (selected == Coefficient.ANGULAR_KA) {
                context.constants.angularKA = Math.max(
                        0.0, context.constants.angularKA + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KV) {
                context.constants.translationalKV = Math.max(
                        0.0, context.constants.translationalKV + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KA) {
                context.constants.translationalKA = Math.max(
                        0.0, context.constants.translationalKA + change
                );
            }
        }

        Pose velocity = context.getFollower().getVelocity();
        double angularVel = velocity.getHeading(AngleUnit.RAD);
        double driveVel = Math.abs(velocity.getX().getIn());

        double time_sec = timer.seconds();
        boolean working = manualState == ManualState.ANGULAR &&
                time_sec < angularProfile.getTotalTime() || manualState == ManualState.DRIVE &&
                time_sec < driveProfile.getTotalTime();

        if (coefficientsChanged && working) {
            context.getFollower().stop();
            manualState = ManualState.IDLE;
            working = false;
        }

        if (!working && manualState != ManualState.IDLE) {
            manualState = ManualState.IDLE;
            context.getFollower().stop();
        }

        if (opMode.gamepad1.xWasPressed() && !working) {
            manualState = ManualState.ANGULAR;
            if (manualAngularHasRun) {
                manualAngularPositive = !manualAngularPositive;
            } else {
                manualAngularHasRun = true;
            }
            resetManualMetrics();
            timer.reset();
            working = true;
        } else if (opMode.gamepad1.yWasPressed() && !working) {
            manualState = ManualState.DRIVE;
            if (manualDriveHasRun) {
                isForward = !isForward;
            } else {
                manualDriveHasRun = true;
            }
            resetManualMetrics();
            timer.reset();
            working = true;
        }

        time_sec = timer.seconds();
        double targetVel = 0.0;
        double currentVel = 0.0;

        switch (working ? manualState : ManualState.IDLE) {
            case IDLE:
                context.getFollower().stop();
                break;
            case ANGULAR:
                double angularDirection = manualAngularPositive ? 1.0 : -1.0;
                targetVel = angularProfile.getVel(time_sec) * angularDirection;
                currentVel = angularVel;
                double angularPow = manualAngularPower(
                        context.constants.angularKV, context.constants.angularKA,
                        context.constants.angularCoeffs.kS,
                        angularProfile.getVel(time_sec),
                        angularProfile.getAccel(time_sec), angularDirection);
                double appliedAngularPower = clipManualPower(angularPow);
                context.getFollower().getDrivetrain().moveWithVectors(
                        0.0, 0.0, appliedAngularPower);
                recordManualFeedforwardSample(targetVel, currentVel,
                        angularPow, appliedAngularPower);
                break;
            case DRIVE:
                targetVel = driveProfile.getVel(time_sec);
                currentVel = driveVel;
                double drivePow =
                        context.constants.translationalKV * targetVel +
                                context.constants.translationalKA * driveProfile.getAccel(time_sec) +
                                context.constants.translationalCoeffs.kS;
                double direction = isForward ? 1.0 : -1.0;
                double rawDrivePower = drivePow * direction;
                double appliedDrivePower = clipManualPower(rawDrivePower);
                context.getFollower().getDrivetrain().moveWithVectors(
                        appliedDrivePower, 0.0, 0.0
                );
                recordManualFeedforwardSample(targetVel, currentVel,
                        rawDrivePower, appliedDrivePower);
                break;
        }

        addTunableValue("Angular KV", context.constants.angularKV,
                selected == Coefficient.ANGULAR_KV);
        addTunableValue("Angular KA", context.constants.angularKA,
                selected == Coefficient.ANGULAR_KA);
        addTunableValue("Translational KV", context.constants.translationalKV,
                selected == Coefficient.TRANSLATIONAL_KV);
        addTunableValue("Translational KA", context.constants.translationalKA,
                selected == Coefficient.TRANSLATIONAL_KA);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addLine("Target Vel: " + number(targetVel));
        context.getTelemetry().addLine("Current Vel: " + number(currentVel) +
                (manualState == ManualState.ANGULAR ? " rad/s" :
                        manualState == ManualState.DRIVE ? " in/s" : ""));
        context.getTelemetry().addData("Velocity error", number(targetVel - currentVel));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Direction", manualState == ManualState.ANGULAR
                    ? (manualAngularPositive ? "COUNTERCLOCKWISE" : "CLOCKWISE")
                    : (isForward ? "FORWARD" : "BACKWARD"));
            context.getTelemetry().addData("RMS velocity error", manualSamples == 0
                    ? Double.NaN : Math.sqrt(manualErrorSquared / manualSamples));
            context.getTelemetry().addData("Peak velocity error", manualPeakError);
            context.getTelemetry().addData("Peak measured velocity", manualPeakVelocity);
            context.getTelemetry().addData("Saturation", manualSamples == 0 ? "0.0%" :
                    Math.round(1000.0 * manualSaturatedSamples / manualSamples) / 10.0 + "%");
            context.getTelemetry().addData("Response CSV", manualCsvPath);
        }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("X: Run and edit angular routine");
        context.getTelemetry().addLine("Y: Run and edit drive routine");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();
        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            if (manualCsv != null) { manualCsv.close(); }
            return true;
        }
        return false;
    }

    private void resetManualMetrics() {
        manualRunNumber++;
        manualSamples = 0;
        manualSaturatedSamples = 0;
        manualErrorSquared = 0.0;
        manualPeakError = 0.0;
        manualPeakVelocity = 0.0;
    }

    private void recordManualFeedforwardSample(double targetVelocity, double actualVelocity,
                                               double rawPower, double appliedPower) {
        if (!Double.isFinite(targetVelocity) || !Double.isFinite(actualVelocity)) { return; }
        double error = targetVelocity - actualVelocity;
        manualSamples++;
        manualErrorSquared += error * error;
        manualPeakError = Math.max(manualPeakError, Math.abs(error));
        manualPeakVelocity = Math.max(manualPeakVelocity, Math.abs(actualVelocity));
        boolean saturated = Math.abs(rawPower) >= 1.0;
        if (saturated) { manualSaturatedSamples++; }
        if (manualCsv != null) {
            String direction = manualState == ManualState.ANGULAR
                    ? (manualAngularPositive ? "COUNTERCLOCKWISE" : "CLOCKWISE")
                    : (isForward ? "FORWARD" : "BACKWARD");
            manualCsv.writeRow(manualRunNumber, timer.seconds(), manualState, direction,
                    targetVelocity, actualVelocity, error, rawPower, appliedPower, saturated);
        }
    }

    static double manualAngularPower(double kV, double kA, double kS,
                                     double profileVelocity, double profileAcceleration,
                                     double direction) {
        double signedDirection = direction >= 0.0 ? 1.0 : -1.0;
        return kV * profileVelocity * signedDirection +
                kA * profileAcceleration * signedDirection + kS * signedDirection;
    }

    static double clipManualPower(double rawPower) {
        return Range.clip(rawPower, -1.0, 1.0);
    }

    @Override
    protected boolean autoTuned() {
        if (autoStage == AutoStage.DONE) { return true; }

        if (autoStage == AutoStage.FAILED) {
            context.getFollower().stop();
            context.getTelemetry().addLine("Feedforward characterization did not pass validation.");
            context.getTelemetry().addData("Validation", validationMessage);
            context.getTelemetry().addLine("The previous feedforward values are still active.");
            context.getTelemetry().addLine("Press A to retry automatically.");
            context.getTelemetry().addLine("Press B to switch to manual tuning.");
            if (context.isDebugMode()) { reportFitDiagnostics(); }
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed()) {
                restartAutomaticCharacterization();
            } else if (opMode.gamepad1.bWasPressed()) {
                manualMode = true;
                init();
            }
            return false;
        }

        if (autoStage == AutoStage.FITTING) {
            angularFit = fitFeedforward(
                    angularObservations, Math.abs(context.constants.angularCoeffs.kS));
            translationalFit = fitFeedforward(
                    translationalObservations,
                    Math.abs(context.constants.translationalCoeffs.kS));

            boolean angularValid = angularFit.isValid();
            boolean translationalValid = translationalFit.isValid();
            if (angularValid && translationalValid) {
                context.constants.angularKV = angularFit.kV;
                context.constants.angularKA = angularFit.kA;
                context.constants.translationalKV = translationalFit.kV;
                context.constants.translationalKA = translationalFit.kA;
            }
            validationMessage = "Angular " + (angularValid ? "PASSED" : "FAILED") +
                    "; Translation " + (translationalValid ? "PASSED" : "FAILED");
            if (!angularValid || !translationalValid) {
                validationMessage += "; no candidate values applied";
            }
            writeCharacterizationCsv();
            autoStage = angularValid && translationalValid ? AutoStage.DONE : AutoStage.FAILED;
            return autoStage == AutoStage.DONE;
        }

        AutoRun run = AUTO_RUNS[autoRunIndex];
        if (autoStage == AutoStage.PROMPT) {
            context.getFollower().stop();
            Pose velocity = context.getFollower().getVelocity();
            boolean stationary = Math.hypot(
                    velocity.getX().getIn(), velocity.getY().getIn()) <=
                    STATIONARY_LINEAR_SPEED_IN_PER_SEC &&
                    Math.abs(velocity.getHeading(AngleUnit.RAD)) <=
                            STATIONARY_ANGULAR_SPEED_RAD_PER_SEC;
            String direction = run.forward
                    ? (run.axis == Axis.ANGULAR ? "counterclockwise" : "forward")
                    : (run.axis == Axis.ANGULAR ? "clockwise" : "backward");
            context.getTelemetry().addLine("Feedforward characterization " +
                    (autoRunIndex + 1) + " / " + AUTO_RUNS.length);
            context.getTelemetry().addData("Test", run.axis + " " + run.excitation);
            context.getTelemetry().addData("Direction", direction);
            context.getTelemetry().addLine(run.axis == Axis.ANGULAR
                    ? "Place the robot where it can rotate safely."
                    : "Point the " + (run.forward ? "front" : "back") +
                            " toward at least 72 inches of clear space.");
            context.getTelemetry().addLine(
                    "Press A when the robot is stationary and the direction is safe.");
            context.getTelemetry().addLine(stationary
                    ? "Robot is stationary and ready."
                    : "Waiting for the robot to stop before starting.");
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed() && stationary) {
                commandPowerFilter.reset();
                lastAppliedCharacterizationPower = 0.0;
                timer.reset();
                autoStage = AutoStage.RUNNING;
            }
            return false;
        }

        // ElapsedTime.time(TimeUnit.SECONDS) returns a whole number of seconds in FTC SDK 11.1.
        // seconds() preserves the sub-second resolution required for a smooth quasistatic ramp.
        double elapsed = timer.seconds();
        if (autoStage == AutoStage.RUNNING) {
            double duration = run.excitation == Excitation.QUASISTATIC
                    ? QUASISTATIC_TIME : DYNAMIC_TIME;
            if (elapsed >= duration) {
                context.getFollower().stop();
                lastAppliedCharacterizationPower = 0.0;
                timer.reset();
                autoStage = AutoStage.SETTLING;
            } else {
                // Preserve the legacy power-window alignment only when the compatibility moving
                // average is selected. Kalman state estimates are current-time estimates and must
                // not be shifted by the old seven-sample window.
                double alignedPower = commandPowerFilter.update(
                        lastAppliedCharacterizationPower).value();
                double power = run.excitation == Excitation.QUASISTATIC
                        ? CHARACTERIZATION_POWER * elapsed / duration
                        : CHARACTERIZATION_POWER;
                double signedPower = run.forward ? power : -power;
                if (run.axis == Axis.ANGULAR) {
                    context.getFollower().getDrivetrain().moveWithVectors(
                            0.0, 0.0, signedPower);
                } else {
                    context.getFollower().getDrivetrain().moveWithVectors(
                            signedPower, 0.0, 0.0);
                }
                lastAppliedCharacterizationPower = power;

                Pose velocity = context.getFollower().getVelocity();
                Pose acceleration = context.getFollower().getAcceleration();
                double rawVelocity = run.axis == Axis.ANGULAR
                        ? velocity.getHeading(AngleUnit.RAD)
                        : velocity.getX().getIn();
                double rawAcceleration = run.axis == Axis.ANGULAR
                        ? acceleration.getHeading(AngleUnit.RAD)
                        : acceleration.getX().getIn();
                if (elapsed >= MIN_SAMPLE_TIME && Double.isFinite(rawAcceleration)) {
                    double measuredVelocity = Math.abs(rawVelocity);
                    // Project acceleration along the measured direction of travel. This remains
                    // correct even when a localizer's positive axis is opposite motor power, and
                    // unlike abs(acceleration), preserves real deceleration samples.
                    double measuredAcceleration = rawAcceleration * Math.signum(rawVelocity);
                    double velocityFloor = run.axis == Axis.ANGULAR ? 0.02 : 0.25;
                    if (measuredVelocity >= velocityFloor) {
                        Observation observation = new Observation(
                                alignedPower, measuredVelocity, measuredAcceleration,
                                autoRunIndex % 4, elapsed);
                        (run.axis == Axis.ANGULAR
                                ? angularObservations : translationalObservations).add(observation);
                    }
                }
            }
        } else if (autoStage == AutoStage.SETTLING) {
            context.getFollower().stop();
            if (elapsed >= AUTO_SETTLE_TIME) {
                autoRunIndex++;
                autoStage = autoRunIndex >= AUTO_RUNS.length
                        ? AutoStage.FITTING : AutoStage.PROMPT;
                timer.reset();
            }
        }

        context.getTelemetry().addLine(feedforwardActionDescription(run));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Characterization", (autoRunIndex + 1) +
                    " / " + AUTO_RUNS.length);
            context.getTelemetry().addData("Test", run.axis + " " + run.excitation);
            context.getTelemetry().addData("Direction", run.forward ? "FORWARD / CCW" : "BACKWARD / CW");
            context.getTelemetry().addData("Angular samples", angularObservations.size());
            context.getTelemetry().addData("Translation samples", translationalObservations.size());
            context.getTelemetry().addData("CSV", csvPath);
        }
        context.getTelemetry().update();
        return false;
    }

    private boolean usesMovingAverageVelocity() {
        return context.getFollower().getLocalizer().getVelocityFilterMode()
                == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE;
    }

    private String feedforwardActionDescription(AutoRun run) {
        if (autoStage == AutoStage.SETTLING) {
            return "Robot is stopping before the next feedforward run.";
        }
        String motion;
        if (run.axis == Axis.ANGULAR) {
            motion = run.forward ? "turning counterclockwise" : "turning clockwise";
        } else {
            motion = run.forward ? "driving forward" : "driving backward";
        }
        return "Robot is " + motion + " for the feedforward test.";
    }

    private void reportFitDiagnostics() {
        reportFitDiagnostics("Angular", angularFit);
        reportFitDiagnostics("Translation", translationalFit);
        context.getTelemetry().addData("Characterization CSV", csvPath);
    }

    private void reportFitDiagnostics(String axis, FitResult fit) {
        if (fit == null) {
            context.getTelemetry().addData(axis + " fit", "Unavailable");
            return;
        }
        context.getTelemetry().addData(axis + " candidate kV", fit.kV);
        context.getTelemetry().addData(axis + " candidate kA", fit.kA);
        context.getTelemetry().addData(axis + " fit RMSE", fit.rmse);
        context.getTelemetry().addData(axis + " cross-validation RMSE", fit.crossValidationRmse);
        context.getTelemetry().addData(axis + " fit R^2", fit.rSquared);
        context.getTelemetry().addData(axis + " samples", fit.sampleCount);
    }

    private void writeCharacterizationCsv() {
        TuningCsvWriter writer = TuningCsvWriter.open(
                "feedforward_characterization",
                "axis", "run", "excitation", "direction", "elapsed_s",
                "commanded_power", "velocity", "acceleration", "predicted_power",
                "residual", "kS", "kV", "kA"
        );
        writeAxisCsv(writer, "ANGULAR", angularObservations,
                Math.abs(context.constants.angularCoeffs.kS), angularFit);
        writeAxisCsv(writer, "TRANSLATIONAL", translationalObservations,
                Math.abs(context.constants.translationalCoeffs.kS), translationalFit);
        writer.close();
        csvPath = writer.getPath();
        csvError = writer.getError();
    }

    private static void writeAxisCsv(TuningCsvWriter writer, String axis,
                                     List<Observation> observations, double kS,
                                     FitResult fit) {
        for (Observation sample : observations) {
            String excitation = sample.run < 2 ? "QUASISTATIC" : "DYNAMIC";
            String direction = sample.run % 2 == 0 ? "POSITIVE" : "NEGATIVE";
            double predicted = fit == null ? Double.NaN :
                    kS + fit.kV * sample.velocity + fit.kA * sample.acceleration;
            writer.writeRow(
                    axis, sample.run + 1, excitation, direction, sample.elapsedSeconds,
                    sample.power, sample.velocity, sample.acceleration, predicted,
                    sample.power - predicted, kS,
                    fit == null ? Double.NaN : fit.kV,
                    fit == null ? Double.NaN : fit.kA
            );
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Angular KV", number(context.constants.angularKV));
        context.getTelemetry().addData("Angular KA", number(context.constants.angularKA));
        context.getTelemetry().addData("Translational KV", number(context.constants.translationalKV));
        context.getTelemetry().addData("Translational KA", number(context.constants.translationalKA));
        context.getTelemetry().addData("Validation", validationMessage);
        if (manualMode) {
            context.getTelemetry().addData("Manual response CSV", manualCsvPath);
            return;
        }
        if (!context.isDebugMode()) { return; }
        context.getTelemetry().addData("Characterization CSV", csvPath);
        if (csvError != null) {
            context.getTelemetry().addData("CSV warning", csvError);
        }
        if (angularFit != null) {
            context.getTelemetry().addData("Angular fit RMSE", angularFit.rmse);
            context.getTelemetry().addData("Angular cross-validation RMSE",
                    angularFit.crossValidationRmse);
            context.getTelemetry().addData("Angular fit R^2", angularFit.rSquared);
            context.getTelemetry().addData("Angular samples", angularFit.sampleCount);
        }
        if (translationalFit != null) {
            context.getTelemetry().addData("Translation fit RMSE", translationalFit.rmse);
            context.getTelemetry().addData("Translation cross-validation RMSE",
                    translationalFit.crossValidationRmse);
            context.getTelemetry().addData("Translation fit R^2", translationalFit.rSquared);
            context.getTelemetry().addData("Translation samples", translationalFit.sampleCount);
        }
    }
}
