package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * Tunes static friction and PD position gains. Static friction is found with a bounded search;
 * PD gains come from a repeatable relay-feedback limit cycle and the Ziegler-Nichols PD rule.
 * The identified gains can then be checked with operator-triggered alternating point-to-point
 * moves before the operator accepts them.
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
        SETTLING_FOR_OPERATOR_CHECK,
        OPERATOR_CHECK
    }

    private static final double MOVEMENT_THRESHOLD = 0.05;
    private static final double HEADING_THRESHOLD = 0.02;
    private static final double GUESS_TIME_MS = 1500.0;
    private static final double SETTLING_TIME_MS = 750.0;
    private static final double RELAY_MIN_TIMEOUT_SECONDS = 16.0;
    private static final double RELAY_MAX_TIMEOUT_SECONDS = 60.0;
    private static final double MAX_RELAY_SAMPLE_GAP_SECONDS = 0.100;
    private static final double TEST_TIMEOUT_SECONDS = 4.0;
    private static final double TEST_SETTLED_SECONDS = 0.50;
    private static final double MAX_TEST_POWER = 0.75;
    private static final double TEST_BREAKAWAY_RESERVE = 0.02;

    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sessionTimer = new ElapsedTime();
    private final PDSController controller;
    private BinarySearch search;
    private final double threshold;

    private PDSState state = PDSState.TUNING_KS;
    private double startValue;
    private RelayOscillationAnalyzer relay;
    private RelayOscillationAnalyzer.Estimate relayEstimate;
    private double testTarget;
    private double nextTestTarget;
    private double testSettledSince = -1.0;
    private double testFinalError = Double.NaN;
    private boolean testActive;
    private int completedTestCount;
    private String operatorCheckSummary = "Not started";
    private TuningCsvWriter csv;
    private double relayDeadlineSeconds = RELAY_MIN_TIMEOUT_SECONDS;
    private double lastRelaySampleSeconds = Double.NaN;

    PDSRoutine(TunerContext context, Axis axis) {
        this.axis = axis;
        controller = new PDSController(new PDSCoefficients());
        if (axis == Axis.HEADING) { controller.setAngularController(); }
        threshold = axis == Axis.HEADING ? HEADING_THRESHOLD : MOVEMENT_THRESHOLD;
    }

    void start(TunerContext context) {
        if (csv != null) { csv.close(); }
        search = new BinarySearch(0.0, 0.4, 0.01);
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
        testActive = false;
        completedTestCount = 0;
        operatorCheckSummary = "Pending relay identification";
        csv = TuningCsvWriter.open(
                "pds_" + axis.toString().toLowerCase(),
                "time_s", "axis", "state", "target", "position", "error",
                "velocity", "command", "relay_cycles", "relay_amplitude",
                "relay_period_s", "relay_amplitude_spread", "relay_period_spread",
                "relay_deadline_s", "relay_usable_cycles", "relay_discarded_cycles"
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

    /** PDS relay and operator-requested test motion are bidirectional, so start at field center. */
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
            case SETTLING_FOR_OPERATOR_CHECK:
                if (settle(context, PDSState.OPERATOR_CHECK)) { beginOperatorCheck(context); }
                return false;
            case OPERATOR_CHECK:
                return updateOperatorCheck(context);
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
        lastRelaySampleSeconds = Double.NaN;
        operatorCheckSummary = "Collecting repeatable relay cycles";
    }

    /**
     * Keeps relay excitation clearly above the measured breakaway command. A fraction of the
     * remaining motor authority is used instead of a small fixed increment: this keeps reversals
     * decisive across drivetrains with different static friction while leaving ample safety
     * reserve below full power.
     */
    static double relayPowerFor(Axis axis, double staticGain) {
        double breakawayPower = Range.clip(Math.abs(staticGain), 0.0, 1.0);
        double usablePowerFraction = axis == Axis.HEADING ? 0.35 : 0.40;
        double minimumPower = axis == Axis.HEADING ? 0.48 : 0.52;
        double excitationPower = breakawayPower +
                usablePowerFraction * (1.0 - breakawayPower);
        return Math.min(0.75, Math.max(minimumPower, excitationPower));
    }

    private boolean updateRelay(TunerContext context) {
        double elapsed = timer.seconds();
        double position = getRelativePosition(context);
        double safetyLimit = axis == Axis.HEADING ? Math.toRadians(80.0) : 18.0;
        if (!Double.isFinite(position) || Math.abs(position) > safetyLimit) {
            abort(context, "Relay test exceeded its bounded travel envelope");
        }

        if (Double.isFinite(lastRelaySampleSeconds) &&
                elapsed - lastRelaySampleSeconds > MAX_RELAY_SAMPLE_GAP_SECONDS) {
            relay.discardCurrentCycle();
        }
        lastRelaySampleSeconds = elapsed;
        relay.observe(elapsed, position);
        double command = relay.getCommand();
        move(context, command);

        relayDeadlineSeconds = relay.recommendedTimeoutSeconds(
                RELAY_MIN_TIMEOUT_SECONDS, RELAY_MAX_TIMEOUT_SECONDS);
        logSample(context, 0.0, position, command);
        boolean timedOut = elapsed >= relayDeadlineSeconds;
        if (!relay.hasStableEstimate() && !timedOut) { return false; }
        if (!relay.canEstimate()) {
            abort(context, "Relay test produced only " + relay.getUsableCycleCount() + " of " +
                    relay.getRequiredCycleCount() + " required usable oscillations" +
                    (relay.getDiscardedCycleCount() == 0 ? "" : " after dropping " +
                            relay.getDiscardedCycleCount() + " loop-stalled cycle(s)"));
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
        state = PDSState.SETTLING_FOR_OPERATOR_CHECK;
        timer.reset();
        operatorCheckSummary = "Relay fit complete; waiting for operator check";
        return false;
    }

    private void beginOperatorCheck(TunerContext context) {
        nextTestTarget = axis == Axis.HEADING ? Math.toRadians(30.0) : 8.0;
        testTarget = 0.0;
        testSettledSince = -1.0;
        testFinalError = Double.NaN;
        testActive = false;
        operatorCheckSummary = "Ready for operator check";
        controller.reset();
    }

    private boolean updateOperatorCheck(TunerContext context) {
        if (!testActive) {
            context.getFollower().stop();
            logSample(context, testTarget, getRelativePosition(context), 0.0);

            if (context.testButtonWasPressed()) {
                testTarget = nextTestTarget;
                nextTestTarget = -nextTestTarget;
                testSettledSince = -1.0;
                testFinalError = testTarget - getRelativePosition(context);
                testActive = true;
                timer.reset();
                controller.reset();
                operatorCheckSummary = "Running operator-requested test";
                return false;
            }
            if (context.retuneButtonWasPressed()) {
                start(context);
                return false;
            }
            if (context.acceptButtonWasPressed()) {
                operatorCheckSummary = "Gains accepted by operator";
                if (csv != null) { csv.close(); }
                return true;
            }
            return false;
        }

        double elapsed = timer.seconds();
        double position = getRelativePosition(context);
        double velocity = getAxisVelocity(context);
        double error = testTarget - position;
        double errorTolerance = axis == Axis.HEADING ? Math.toRadians(2.5) : 0.75;
        double velocityTolerance = axis == Axis.HEADING ? 0.10 : 1.0;
        double command = ensureTestBreakawayPower(
                controller.calculate(error), error, velocity,
                controller.getCoefficients().kS, errorTolerance, velocityTolerance);
        command = Range.clip(command, -MAX_TEST_POWER, MAX_TEST_POWER);

        double safetyLimit = axis == Axis.HEADING ? Math.toRadians(80.0) : 18.0;
        if (!Double.isFinite(position) || !Double.isFinite(velocity) ||
                Math.abs(position) > safetyLimit) {
            abort(context, "Operator-requested PDS test exceeded its bounded travel envelope");
        }

        move(context, command);
        testFinalError = error;
        logSample(context, testTarget, position, command);

        boolean withinTolerance = Math.abs(error) <= errorTolerance &&
                Math.abs(velocity) <= velocityTolerance;
        if (withinTolerance) {
            if (testSettledSince < 0.0) { testSettledSince = elapsed; }
        } else {
            testSettledSince = -1.0;
        }

        boolean settled = testSettledSince >= 0.0 &&
                elapsed - testSettledSince >= TEST_SETTLED_SECONDS;
        if (!settled && elapsed < TEST_TIMEOUT_SECONDS) { return false; }

        context.getFollower().stop();
        testActive = false;
        completedTestCount++;
        operatorCheckSummary = settled
                ? "Test complete; inspect the response and run again or accept"
                : "Test stopped after timeout; retune if the response is not acceptable";
        return false;
    }

    /**
     * The PDS controller deliberately softens its static-friction term near zero error. If the
     * robot has stopped just outside the test tolerance, that softened output can fall below
     * the measured breakaway command and leave an otherwise valid controller permanently stuck.
     * Apply the same narrow endpoint recovery used by the follower: only a stalled robot outside
     * tolerance receives enough power to move, and no floor is applied after it reaches tolerance.
     */
    static double ensureTestBreakawayPower(double requestedPower, double error,
                                           double velocity, double staticGain,
                                           double errorTolerance,
                                           double velocityTolerance) {
        if (!Double.isFinite(requestedPower) || !Double.isFinite(error) ||
                !Double.isFinite(velocity) || !Double.isFinite(staticGain) ||
                !Double.isFinite(errorTolerance) || errorTolerance < 0.0 ||
                !Double.isFinite(velocityTolerance) || velocityTolerance < 0.0) {
            return requestedPower;
        }
        boolean outsideTolerance = Math.abs(error) > errorTolerance;
        boolean stalled = Math.abs(velocity) < velocityTolerance;
        if (!outsideTolerance || !stalled) { return requestedPower; }

        double minimumPower = Math.min(MAX_TEST_POWER,
                Math.abs(staticGain) + TEST_BREAKAWAY_RESERVE);
        if (Math.abs(requestedPower) >= minimumPower) { return requestedPower; }
        return Math.copySign(minimumPower, error);
    }

    private void abort(TunerContext context, String reason) {
        context.getFollower().stop();
        operatorCheckSummary = "FAILED: " + reason;
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
                relay == null ? "" : relayDeadlineSeconds,
                relay == null ? 0 : relay.getUsableCycleCount(),
                relay == null ? 0 : relay.getDiscardedCycleCount()
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
        context.getTelemetry().addLine(actionDescription(context));
        if (!context.isDebugMode()) {
            context.getTelemetry().update();
            return;
        }
        context.getTelemetry().addData("Step", state.toString().replace('_', ' '));
        context.getTelemetry().addData("Static guess", search.getGuess());
        if (relay != null) {
            context.getTelemetry().addData("Usable relay cycles",
                    relay.getUsableCycleCount() + " / " + relay.getRequiredCycleCount());
            context.getTelemetry().addData("Cycles dropped after loop stalls",
                    relay.getDiscardedCycleCount());
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
        if (state == PDSState.OPERATOR_CHECK) {
            context.getTelemetry().addData("Operator tests completed", completedTestCount);
            context.getTelemetry().addData("Current test target", testTarget);
            context.getTelemetry().addData("Current test error", testFinalError);
        }
        context.getTelemetry().addData("CSV", getCsvPath());
        context.getTelemetry().addLine(state == PDSState.OPERATOR_CHECK
                ? "The robot remains stopped until you request or accept a test."
                : "Keep the OpMode running until identification finishes.");
        context.getTelemetry().update();
    }

    private String actionDescription(TunerContext context) {
        switch (state) {
            case TUNING_KS:
                return axis == Axis.HEADING
                        ? "Robot is finding the minimum power needed to turn."
                        : "Robot is finding the minimum power needed to move.";
            case SETTLING_BETWEEN_KS:
            case SETTLING_FOR_RELAY:
            case SETTLING_FOR_OPERATOR_CHECK:
                return "Robot is stopping before the next test.";
            case TUNING_RELAY:
                return axis == Axis.HEADING
                        ? "Robot is turning back and forth to identify its response."
                        : "Robot is driving back and forth to identify its response.";
            case OPERATOR_CHECK:
                if (testActive) {
                    return axis == Axis.HEADING
                            ? "Robot is turning to the requested test target."
                            : "Robot is driving to the requested test target.";
                }
                return operatorCheckSummary + ". Press " + context.control("X") +
                        " to run the next alternating test, " + context.control("A") +
                        " to accept, or " + context.control("B") + " to retune.";
            default:
                return "Robot is running the controller test.";
        }
    }

    PDSCoefficients getCoefficients() { return controller.getCoefficients(); }

    String getOperatorCheckSummary() { return operatorCheckSummary; }

    String getCsvPath() { return csv == null ? "Unavailable" : csv.getPath(); }
}
