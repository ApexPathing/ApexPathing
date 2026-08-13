package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import geometry.AngleUnit;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

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
    private static final double SIM_STAGING_OFFSET = 55.0;

    private enum Coefficient { ANGULAR_KV, ANGULAR_KA, TRANSLATIONAL_KV, TRANSLATIONAL_KA }

    private enum ManualState { IDLE, ANGULAR, DRIVE }
    private ManualState manualState = ManualState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private TrapezoidProfile driveProfile;
    private TrapezoidProfile angularProfile;

    private Coefficient selected = Coefficient.ANGULAR_KV;

    private enum Axis { ANGULAR, TRANSLATIONAL }
    private enum Excitation { QUASISTATIC, DYNAMIC }
    private enum AutoStage { PROMPT, RUNNING, SETTLING, FITTING, DONE }

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
                    Double.isFinite(crossValidationRmse) && crossValidationRmse <= 0.15 &&
                    Double.isFinite(rSquared) && rSquared >= 0.70;
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
        if (manualMode) {
            manualState = ManualState.IDLE;
            context.getFollower().stop();
        } else {
            angularObservations.clear();
            translationalObservations.clear();
            angularFit = null;
            translationalFit = null;
            validationMessage = "Collecting characterization data";
            csvPath = "Pending";
            csvError = null;
            autoRunIndex = 0;
            autoStage = AutoStage.PROMPT;
        }
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
        double targetSum = 0.0;
        for (Observation sample : samples) {
            double residual = predictionError(sample, kS, fit);
            squaredError += residual * residual;
            targetSum += sample.power - kS;
        }
        double rmse = Math.sqrt(squaredError / samples.size());
        double targetMean = targetSum / samples.size();
        double totalSquaredError = 0.0;
        for (Observation sample : samples) {
            double centered = (sample.power - kS) - targetMean;
            totalSquaredError += centered * centered;
        }
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

        if (opMode.gamepad1.aWasPressed()) {
            return true;
        }

        Pose velocity = context.getFollower().getVelocity();
        double angularVel = velocity.getHeading(AngleUnit.RAD);
        double driveVel = Math.abs(velocity.getX().getIn());

        double time_sec = timer.time(TimeUnit.SECONDS);
        boolean working = manualState == ManualState.ANGULAR &&
                time_sec < angularProfile.getTotalTime() || manualState == ManualState.DRIVE &&
                time_sec < driveProfile.getTotalTime();

        if (!working && manualState != ManualState.IDLE) {
            manualState = ManualState.IDLE;
            context.getFollower().stop();
        }

        if (opMode.gamepad1.xWasPressed() && !working) {
            manualState = ManualState.ANGULAR;
            timer.reset();
            working = true;
        } else if (opMode.gamepad1.yWasPressed() && !working) {
            manualState = ManualState.DRIVE;
            if (manualDriveHasRun) {
                isForward = !isForward;
            } else {
                manualDriveHasRun = true;
            }
            timer.reset();
            working = true;
        }

        time_sec = timer.time(TimeUnit.SECONDS);
        double targetVel = 0.0;
        double currentVel = 0.0;

        switch (working ? manualState : ManualState.IDLE) {
            case IDLE:
                context.getFollower().stop();
                break;
            case ANGULAR:
                targetVel = angularProfile.getVel(time_sec);
                currentVel = angularVel;
                double angularPow =
                        context.constants.angularKV * targetVel +
                                context.constants.angularKA * angularProfile.getAccel(time_sec) +
                                context.constants.angularCoeffs.kS;
                context.getFollower().getDrivetrain().moveWithVectors(0.0, 0.0, angularPow);
                break;
            case DRIVE:
                targetVel = driveProfile.getVel(time_sec);
                currentVel = driveVel;
                double drivePow =
                        context.constants.translationalKV * targetVel +
                                context.constants.translationalKA * driveProfile.getAccel(time_sec) +
                                context.constants.translationalCoeffs.kS;
                double direction = isForward ? 1.0 : -1.0;
                context.getFollower().getDrivetrain().moveWithVectors(
                        drivePow * direction, 0.0, 0.0
                );
                break;
        }

        context.getTelemetry().addData("Selected", selected.toString());
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("Dpad Left/Right: Change increment");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("Target Vel: " + targetVel);
        context.getTelemetry().addLine("Current Vel: " + currentVel +
                (manualState == ManualState.ANGULAR ? " rad/s" :
                        manualState == ManualState.DRIVE ? " in/s" : ""));
        if (manualState == ManualState.DRIVE) {
            context.getTelemetry().addData("Drive direction", isForward ? "FORWARD" : "BACKWARD");
        }
        if (working) {
            context.getTelemetry().addLine("Current velocity " +
                    ((currentVel >= targetVel) ? "> target" : "< target"));
        } else {
            context.getTelemetry().addData("Routine", "IDLE");
        }
        context.getTelemetry().addLine(control("X") + ": Run and edit angular routine");
        context.getTelemetry().addLine(control("Y") + ": Run and edit drive routine");
        context.getTelemetry().addLine(control("A") + ": Save");
        context.getTelemetry().update();
        return false;
    }

    @Override
    protected boolean autoTuned() {
        if (autoStage == AutoStage.DONE) { return true; }

        if (autoStage == AutoStage.FITTING) {
            angularFit = fitFeedforward(
                    angularObservations, Math.abs(context.constants.angularCoeffs.kS));
            translationalFit = fitFeedforward(
                    translationalObservations,
                    Math.abs(context.constants.translationalCoeffs.kS));

            boolean angularValid = angularFit.isValid();
            boolean translationalValid = translationalFit.isValid();
            if (angularValid) {
                context.constants.angularKV = angularFit.kV;
                context.constants.angularKA = angularFit.kA;
            }
            if (translationalValid) {
                context.constants.translationalKV = translationalFit.kV;
                context.constants.translationalKA = translationalFit.kA;
            }
            validationMessage = "Angular " + (angularValid ? "PASSED" : "FAILED - kept prior values") +
                    "; Translation " +
                    (translationalValid ? "PASSED" : "FAILED - kept prior values");
            writeCharacterizationCsv();
            autoStage = AutoStage.DONE;
            return true;
        }

        AutoRun run = AUTO_RUNS[autoRunIndex];
        if (autoStage == AutoStage.PROMPT) {
            context.getFollower().stop();
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
            context.getTelemetry().addLine("Press " + control("A") +
                    " when the robot is stationary and the direction is safe.");
            context.getTelemetry().update();
            if (opMode.gamepad1.aWasPressed()) {
                double stagingX = run.axis == Axis.ANGULAR ? 0.0 :
                        (run.forward ? -SIM_STAGING_OFFSET : SIM_STAGING_OFFSET);
                positionRobotForSimulation(new Pose(
                        Vector.of(stagingX, 0.0, DistUnit.IN), Angle.fromRad(0.0)));
                timer.reset();
                autoStage = AutoStage.RUNNING;
            }
            return false;
        }

        double elapsed = timer.time(TimeUnit.SECONDS);
        if (autoStage == AutoStage.RUNNING) {
            double duration = run.excitation == Excitation.QUASISTATIC
                    ? QUASISTATIC_TIME : DYNAMIC_TIME;
            if (elapsed >= duration) {
                context.getFollower().stop();
                timer.reset();
                autoStage = AutoStage.SETTLING;
            } else {
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

                if (elapsed >= MIN_SAMPLE_TIME) {
                    Pose velocity = context.getFollower().getVelocity();
                    Pose acceleration = context.getFollower().getAcceleration();
                    double rawVelocity = run.axis == Axis.ANGULAR
                            ? velocity.getHeading(AngleUnit.RAD)
                            : velocity.getX().getIn();
                    double rawAcceleration = run.axis == Axis.ANGULAR
                            ? acceleration.getHeading(AngleUnit.RAD)
                            : acceleration.getX().getIn();
                    double measuredVelocity = Math.abs(rawVelocity);
                    // Project acceleration along the measured direction of travel. This remains
                    // correct even when a localizer's positive axis is opposite motor power, and
                    // unlike abs(acceleration), preserves real deceleration samples.
                    double measuredAcceleration = rawAcceleration * Math.signum(rawVelocity);
                    double velocityFloor = run.axis == Axis.ANGULAR ? 0.02 : 0.25;
                    if (measuredVelocity >= velocityFloor) {
                        Observation observation = new Observation(
                                power, measuredVelocity, measuredAcceleration,
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
        context.getTelemetry().addData("Angular KV", context.constants.angularKV);
        context.getTelemetry().addData("Angular KA", context.constants.angularKA);
        context.getTelemetry().addData("Translational KV", context.constants.translationalKV);
        context.getTelemetry().addData("Translational KA", context.constants.translationalKA);
        context.getTelemetry().addData("Validation", validationMessage);
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
