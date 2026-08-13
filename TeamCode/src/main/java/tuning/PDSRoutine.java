package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * Tunes static friction and PD position gains. Static friction is found with a bounded search;
 * PD gains come from a repeatable relay-feedback limit cycle and the Ziegler-Nichols PD rule.
 * A final point-to-point response is logged and checked before the phase reports completion.
 */
public class PDSRoutine {
    enum Axis {
        DRIVE,
        STRAFE,
        HEADING
    }

    enum PDSState {
        TUNING_KS,
        SETTLING_BETWEEN_KS,
        SETTLING_FOR_RELAY,
        TUNING_RELAY,
        SETTLING_FOR_VALIDATION,
        VALIDATING_PD
    }

    private static final double MOVEMENT_THRESHOLD = 0.05;
    private static final double HEADING_THRESHOLD = 0.02;
    private static final double GUESS_TIME_MS = 1500.0;
    private static final double SETTLING_TIME_MS = 750.0;
    private static final double RELAY_MIN_TIMEOUT_SECONDS = 16.0;
    private static final double RELAY_MAX_TIMEOUT_SECONDS = 60.0;
    private static final double VALIDATION_TIMEOUT_SECONDS = 4.0;
    private static final double VALIDATION_SETTLED_SECONDS = 0.50;
    private static final double MAX_VALIDATION_POWER = 0.75;

    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sessionTimer = new ElapsedTime();
    private final PDSController controller;
    private final BinarySearch search;
    private final double threshold;

    private PDSState state = PDSState.TUNING_KS;
    private double startValue;
    private RelayOscillationAnalyzer relay;
    private RelayOscillationAnalyzer.Estimate relayEstimate;
    private double validationTarget;
    private double validationMaxPosition;
    private double validationSettledSince = -1.0;
    private double validationFinalError = Double.NaN;
    private double validationOvershoot = Double.NaN;
    private boolean validationPassed;
    private String validationSummary = "Not run";
    private TuningCsvWriter csv;
    private double relayDeadlineSeconds = RELAY_MIN_TIMEOUT_SECONDS;

    PDSRoutine(TunerContext context, Axis axis) {
        search = new BinarySearch(0.0, 0.4, 0.01);
        this.axis = axis;
        controller = new PDSController(new PDSCoefficients());
        if (axis == Axis.HEADING) { controller.setAngularController(); }
        threshold = axis == Axis.HEADING ? HEADING_THRESHOLD : MOVEMENT_THRESHOLD;
    }

    void start(TunerContext context) {
        context.getFollower().disableControllers();
        resetAxisPose(context);
        timer.reset();
        sessionTimer.reset();
        controller.getCoefficients().setkP(0.0);
        controller.getCoefficients().setkD(0.0);
        controller.getCoefficients().setkS(search.getGuess());
        state = PDSState.TUNING_KS;
        relay = null;
        relayEstimate = null;
        validationSummary = "Pending relay identification";
        validationPassed = false;
        csv = TuningCsvWriter.open(
                "pds_" + axis.toString().toLowerCase(),
                "time_s", "axis", "state", "target", "position", "error",
                "velocity", "command", "relay_cycles", "relay_amplitude",
                "relay_period_s", "relay_amplitude_spread", "relay_period_spread",
                "relay_deadline_s"
        );
    }

    private void resetAxisPose(TunerContext context) {
        Pose stagingPose = stagingPoseFor(axis);
        if (Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            context.positionRobotForSimulation(stagingPose);
        } else {
            // Re-zeroing odometry does not move the real robot and keeps every bounded test
            // centered on its actual starting location.
            context.getFollower().setPose(Pose.zero());
        }
        // Use the requested reset value instead of a potentially one-update-old hardware cache.
        startValue = getValue(stagingPose);
    }

    /** PDS relay and validation motion are bidirectional, so every axis starts at field center. */
    static Pose stagingPoseFor(Axis ignored) { return Pose.zero(); }

    private void move(TunerContext context, double power) {
        switch (axis) {
            case DRIVE:
                context.getFollower().getDrivetrain().moveWithVectors(power, 0.0, 0.0);
                break;
            case STRAFE:
                context.getFollower().getDrivetrain().moveWithVectors(0.0, power, 0.0);
                break;
            case HEADING:
                context.getFollower().getDrivetrain().moveWithVectors(0.0, 0.0, power);
                break;
        }
    }

    private double getValue(Pose pose) {
        switch (axis) {
            case DRIVE:
                return pose.getX().getIn();
            case STRAFE:
                return pose.getY().getIn();
            case HEADING:
                return pose.getHeading().getRad();
            default:
                return 0.0;
        }
    }

    private double getRelativePosition(TunerContext context) {
        return getValue(context.getFollower().getPose()) - startValue;
    }

    private double getAxisVelocity(TunerContext context) {
        return getValue(context.getFollower().getVelocity());
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean update(TunerContext context) {
        switch (state) {
            case TUNING_KS:
                return updateStaticFriction(context);
            case SETTLING_BETWEEN_KS:
                settle(context, PDSState.TUNING_KS);
                return false;
            case SETTLING_FOR_RELAY:
                if (settle(context, PDSState.TUNING_RELAY)) { beginRelayTest(context); }
                return false;
            case TUNING_RELAY:
                return updateRelay(context);
            case SETTLING_FOR_VALIDATION:
                if (settle(context, PDSState.VALIDATING_PD)) { beginValidation(context); }
                return false;
            case VALIDATING_PD:
                return updateValidation(context);
            default:
                return false;
        }
    }

    private boolean updateStaticFriction(TunerContext context) {
        double command = search.getGuess();
        move(context, command);
        double movement = Math.abs(getRelativePosition(context));
        logSample(context, 0.0, getRelativePosition(context), command);

        // Stop a successful guess as soon as real movement is established. Waiting out the whole
        // window would make every kS probe travel unnecessarily far across the field.
        boolean moved = movement > threshold;
        if (!moved && timer.milliseconds() < GUESS_TIME_MS) { return false; }

        boolean keepTuning = search.updateGuess(!moved);
        state = keepTuning ? PDSState.SETTLING_BETWEEN_KS : PDSState.SETTLING_FOR_RELAY;
        if (!keepTuning) { controller.getCoefficients().setkS(search.getGuess()); }
        timer.reset();
        return false;
    }

    /** Returns true on the loop where the requested next state is entered. */
    private boolean settle(TunerContext context, PDSState nextState) {
        context.getFollower().stop();
        logSample(context, 0.0, getRelativePosition(context), 0.0);
        if (timer.milliseconds() < SETTLING_TIME_MS) { return false; }

        resetAxisPose(context);
        state = nextState;
        timer.reset();
        return true;
    }

    private void beginRelayTest(TunerContext context) {
        double relayPower = relayPowerFor(axis, controller.getCoefficients().kS);
        double hysteresis = axis == Axis.HEADING ? Math.toRadians(4.0) : 1.5;
        relay = new RelayOscillationAnalyzer(relayPower, hysteresis);
        relayDeadlineSeconds = RELAY_MIN_TIMEOUT_SECONDS;
        validationSummary = "Collecting repeatable relay cycles";
    }

    /**
     * Keeps relay excitation clearly above the measured breakaway command. Translational relay
     * motion needs more reserve than the kS search itself: a command that only barely starts the
     * wheels can repeatedly fall back into static friction after each reversal, producing a very
     * slow stick-slip cycle whose period is not useful for controller identification.
     */
    static double relayPowerFor(Axis axis, double staticGain) {
        double basePower = axis == Axis.HEADING ? 0.38 : 0.40;
        double excitationMargin = axis == Axis.HEADING ? 0.16 : 0.22;
        return Math.min(0.65, Math.max(basePower, Math.abs(staticGain) + excitationMargin));
    }

    private boolean updateRelay(TunerContext context) {
        double elapsed = timer.seconds();
        double position = getRelativePosition(context);
        double safetyLimit = axis == Axis.HEADING ? Math.toRadians(80.0) : 18.0;
        if (!Double.isFinite(position) || Math.abs(position) > safetyLimit) {
            abort(context, "Relay test exceeded its bounded travel envelope");
        }

        relay.observe(elapsed, position);
        double command = relay.getCommand();
        move(context, command);

        relayDeadlineSeconds = relay.recommendedTimeoutSeconds(
                RELAY_MIN_TIMEOUT_SECONDS, RELAY_MAX_TIMEOUT_SECONDS);
        logSample(context, 0.0, position, command);
        boolean timedOut = elapsed >= relayDeadlineSeconds;
        if (!relay.hasStableEstimate() && !timedOut) { return false; }
        if (relay.getCycleCount() < relay.getRequiredCycleCount()) {
            abort(context, "Relay test produced only " + relay.getCycleCount() + " of " +
                    relay.getRequiredCycleCount() + " required complete oscillations");
        }
        if (!relay.hasStableEstimate()) {
            RelayOscillationAnalyzer.Estimate candidate = relay.estimate();
            abort(context, "Relay oscillations were not repeatable enough to tune safely " +
                    "(amplitude IQR/median " + percent(candidate.amplitudeRelativeCentralSpread) +
                    ", period IQR/median " + percent(candidate.periodRelativeCentralSpread) +
                    "; limits 15.0% and 12.0%)");
        }

        relayEstimate = relay.estimate();
        PDSCoefficients pd = calculateZieglerNicholsPd(
                relayEstimate.ultimateGain, relayEstimate.periodSeconds);
        controller.getCoefficients().setkP(pd.kP);
        controller.getCoefficients().setkD(pd.kD);
        context.getFollower().stop();
        state = PDSState.SETTLING_FOR_VALIDATION;
        timer.reset();
        validationSummary = "Relay fit complete; validating point-to-point response";
        return false;
    }

    private void beginValidation(TunerContext context) {
        validationTarget = axis == Axis.HEADING ? Math.toRadians(30.0) : 8.0;
        validationMaxPosition = 0.0;
        validationSettledSince = -1.0;
        validationFinalError = validationTarget;
        validationOvershoot = 0.0;
        controller.reset();
    }

    private boolean updateValidation(TunerContext context) {
        double elapsed = timer.seconds();
        double position = getRelativePosition(context);
        double velocity = getAxisVelocity(context);
        double error = validationTarget - position;
        double command = Range.clip(controller.calculate(error),
                -MAX_VALIDATION_POWER, MAX_VALIDATION_POWER);

        double safetyLimit = axis == Axis.HEADING ? Math.toRadians(80.0) : 18.0;
        if (!Double.isFinite(position) || !Double.isFinite(velocity) ||
                Math.abs(position) > safetyLimit) {
            abort(context, "PD validation exceeded its bounded travel envelope");
        }

        move(context, command);
        validationMaxPosition = Math.max(validationMaxPosition, position);
        validationFinalError = error;
        validationOvershoot = Math.max(0.0,
                (validationMaxPosition - validationTarget) / validationTarget);
        logSample(context, validationTarget, position, command);

        double errorTolerance = axis == Axis.HEADING ? Math.toRadians(2.5) : 0.75;
        double velocityTolerance = axis == Axis.HEADING ? 0.10 : 1.0;
        boolean withinTolerance = Math.abs(error) <= errorTolerance &&
                Math.abs(velocity) <= velocityTolerance;
        if (withinTolerance) {
            if (validationSettledSince < 0.0) { validationSettledSince = elapsed; }
        } else {
            validationSettledSince = -1.0;
        }

        boolean settled = validationSettledSince >= 0.0 &&
                elapsed - validationSettledSince >= VALIDATION_SETTLED_SECONDS;
        if (!settled && elapsed < VALIDATION_TIMEOUT_SECONDS) { return false; }

        context.getFollower().stop();
        validationPassed = settled && validationOvershoot <= 0.35;
        validationSummary = validationPassed
                ? "PASSED: settled response, overshoot " +
                        Math.round(validationOvershoot * 1000.0) / 10.0 + "%"
                : "FAILED: response did not meet settling/overshoot limits";
        if (csv != null) { csv.close(); }
        if (!validationPassed) {
            throw new IllegalStateException(
                    "Identified PD gains failed the bounded point-to-point validation and were " +
                            "not saved. Inspect CSV: " + getCsvPath()
            );
        }
        return true;
    }

    private void abort(TunerContext context, String reason) {
        context.getFollower().stop();
        validationSummary = "FAILED: " + reason;
        if (csv != null) { csv.close(); }
        throw new IllegalStateException(reason + ". See PDS CSV: " + getCsvPath());
    }

    private void logSample(TunerContext context, double target, double position, double command) {
        if (csv == null) { return; }
        double velocity = getAxisVelocity(context);
        RelayOscillationAnalyzer.Estimate candidate =
                relay != null && relay.canEstimate() ? relay.estimate() : null;
        csv.writeRow(
                sessionTimer.seconds(), axis, state, target, position,
                target - position, velocity, command,
                relay == null ? 0 : relay.getCycleCount(),
                candidate == null ? "" : candidate.amplitude,
                candidate == null ? "" : candidate.periodSeconds,
                candidate == null ? "" : candidate.amplitudeRelativeCentralSpread,
                candidate == null ? "" : candidate.periodRelativeCentralSpread,
                relay == null ? "" : relayDeadlineSeconds
        );
    }

    private static String percent(double fraction) {
        return Math.round(fraction * 1000.0) / 10.0 + "%";
    }

    /** Classic closed-loop Ziegler-Nichols PD settings from ultimate gain and period. */
    static PDSCoefficients calculateZieglerNicholsPd(double ultimateGain,
                                                     double ultimatePeriodSeconds) {
        if (!Double.isFinite(ultimateGain) || ultimateGain <= 0.0 ||
                !Double.isFinite(ultimatePeriodSeconds) || ultimatePeriodSeconds <= 0.0) {
            throw new IllegalArgumentException(
                    "Ultimate gain and period must be finite and positive");
        }
        double kP = 0.8 * ultimateGain;
        double kD = kP * ultimatePeriodSeconds / 8.0;
        return new PDSCoefficients(kP, kD, 0.0);
    }

    void reportProgress(TunerContext context) {
        context.getTelemetry().addLine("Automatic " + axis.toString().toLowerCase() +
                " tuning in progress");
        context.getTelemetry().addLine(actionDescription());
        if (!context.isDebugMode()) {
            context.getTelemetry().update();
            return;
        }
        context.getTelemetry().addData("Step", state.toString().replace('_', ' '));
        context.getTelemetry().addData("Static guess", search.getGuess());
        if (relay != null) {
            context.getTelemetry().addData("Complete relay cycles",
                    relay.getCycleCount() + " / " + relay.getRequiredCycleCount());
            context.getTelemetry().addData("Adaptive relay deadline",
                    relayDeadlineSeconds + " s");
            if (relay.canEstimate()) {
                RelayOscillationAnalyzer.Estimate candidate = relay.estimate();
                context.getTelemetry().addData("Relay amplitude", candidate.amplitude);
                context.getTelemetry().addData("Relay period", candidate.periodSeconds);
                context.getTelemetry().addData("Amplitude stability",
                        percent(candidate.amplitudeRelativeCentralSpread) + " / 15.0%");
                context.getTelemetry().addData("Period stability",
                        percent(candidate.periodRelativeCentralSpread) + " / 12.0%");
            }
        }
        if (relayEstimate != null) {
            context.getTelemetry().addData("Ultimate gain", relayEstimate.ultimateGain);
            context.getTelemetry().addData("Ultimate period", relayEstimate.periodSeconds);
        }
        if (state == PDSState.VALIDATING_PD) {
            context.getTelemetry().addData("Validation target", validationTarget);
            context.getTelemetry().addData("Validation error", validationFinalError);
            context.getTelemetry().addData("Validation overshoot", validationOvershoot);
        }
        context.getTelemetry().addData("CSV", getCsvPath());
        context.getTelemetry().addLine("Keep the OpMode running until results appear.");
        context.getTelemetry().update();
    }

    private String actionDescription() {
        switch (state) {
            case TUNING_KS:
                return axis == Axis.HEADING
                        ? "Robot is finding the minimum power needed to turn."
                        : "Robot is finding the minimum power needed to move.";
            case SETTLING_BETWEEN_KS:
            case SETTLING_FOR_RELAY:
            case SETTLING_FOR_VALIDATION:
                return "Robot is stopping before the next test.";
            case TUNING_RELAY:
                return axis == Axis.HEADING
                        ? "Robot is turning back and forth to identify its response."
                        : "Robot is driving back and forth to identify its response.";
            case VALIDATING_PD:
                return axis == Axis.HEADING
                        ? "Robot is turning to a target to validate the controller."
                        : "Robot is moving to a target to validate the controller.";
            default:
                return "Robot is running the controller test.";
        }
    }

    PDSCoefficients getCoefficients() { return controller.getCoefficients(); }

    String getValidationSummary() { return validationSummary; }

    boolean validationPassed() { return validationPassed; }

    String getCsvPath() { return csv == null ? "Unavailable" : csv.getPath(); }
}
