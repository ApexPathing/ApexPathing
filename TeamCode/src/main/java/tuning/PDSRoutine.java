package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * Tunes static friction and PD position gains. Static friction is found with a bounded search;
 * kP and kD are then refined with repeated closed-loop point-to-point tests and finite-difference
 * gradient updates. The best measured gains can be checked with operator-triggered alternating
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
        SETTLING_FOR_PD_TEST,
        TUNING_PD,
        SETTLING_FOR_OPERATOR_CHECK,
        OPERATOR_CHECK
    }

    private enum Evaluation {
        BASELINE,
        KP_PLUS,
        KP_MINUS,
        KD_PLUS,
        KD_MINUS,
        CANDIDATE
    }

    private static final double MOVEMENT_THRESHOLD = 0.05;
    private static final double HEADING_THRESHOLD = 0.02;
    private static final double GUESS_TIME_MS = 1500.0;
    private static final double SETTLING_TIME_MS = 750.0;
    private static final double PD_SETTLING_TIME_MS = 400.0;
    private static final double TEST_TIMEOUT_SECONDS = 4.0;
    private static final double OPERATOR_TEST_TIMEOUT_SECONDS = 8.0;
    private static final double TEST_SETTLED_SECONDS = 0.50;
    private static final double MAX_TEST_POWER = 0.75;
    private static final double TEST_BREAKAWAY_RESERVE = 0.02;
    private static final double MAX_PD_SAMPLE_GAP_SECONDS = 0.20;
    private static final int MAX_ERROR_ZERO_CROSSINGS = 8;
    private static final double MAX_SATURATION_FRACTION = 0.80;

    /*
     * Public tuning limits are deliberately simple source-level configuration. Gains are always
     * clipped to these bounds, including saved starting values and finite-difference probes.
     */
    public static double TRANSLATIONAL_KP_MIN = 0.01;
    public static double TRANSLATIONAL_KP_MAX = 0.75;
    public static double TRANSLATIONAL_KD_MIN = 0.0;
    public static double TRANSLATIONAL_KD_MAX = 0.30;
    public static double HEADING_KP_MIN = 0.10;
    public static double HEADING_KP_MAX = 8.0;
    public static double HEADING_KD_MIN = 0.0;
    public static double HEADING_KD_MAX = 2.0;

    public static double TRANSLATIONAL_INITIAL_KP = 0.08;
    public static double TRANSLATIONAL_INITIAL_KD = 0.02;
    public static double HEADING_INITIAL_KP = 0.80;
    public static double HEADING_INITIAL_KD = 0.10;

    public static int PD_TRIAL_REPEATS = 2;
    public static int PD_MAX_ITERATIONS = 5;
    public static int PD_CONVERGENCE_PATIENCE = 2;
    public static double PD_FINITE_DIFFERENCE_FRACTION = 0.05;
    public static double PD_INITIAL_LEARNING_RATE = 0.08;
    public static double PD_MIN_LEARNING_RATE = 0.005;
    public static double PD_MAX_LEARNING_RATE = 0.15;
    public static double PD_RELATIVE_IMPROVEMENT_TOLERANCE = 0.01;
    public static double PD_NORMALIZED_GRADIENT_TOLERANCE = 0.03;
    public static double PD_MAX_TUNING_SECONDS = 240.0;

    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sessionTimer = new ElapsedTime();
    private final PDSController controller;
    private BinarySearch search;
    private final double threshold;

    private PDSState state = PDSState.TUNING_KS;
    private double startValue;
    private double testTarget;
    private double nextTestTarget;
    private double testSettledSince = -1.0;
    private double testFinalError = Double.NaN;
    private boolean testActive;
    private int completedTestCount;
    private String operatorCheckSummary = "Not started";
    private TuningCsvWriter csv;

    private Evaluation evaluation;
    private int iteration;
    private int trialRepeat;
    private double evaluationKp;
    private double evaluationKd;
    private double evaluationCostSum;
    private int evaluationBadTrials;
    private double baselineCost = Double.NaN;
    private double kpPlusCost = Double.NaN;
    private double kpMinusCost = Double.NaN;
    private double kdPlusCost = Double.NaN;
    private double kdMinusCost = Double.NaN;
    private double candidateKp;
    private double candidateKd;
    private double currentKp;
    private double currentKd;
    private double bestKp;
    private double bestKd;
    private double bestCost = Double.POSITIVE_INFINITY;
    private double normalizedGradientP = Double.NaN;
    private double normalizedGradientD = Double.NaN;
    private double normalizedGradientMagnitude = Double.NaN;
    private boolean finiteDifferenceTrialsSafe;
    private double learningRate;
    private int smallGradientIterations;
    private int smallImprovementIterations;
    private double pdTuningStartedSeconds;
    private String automaticTuneSummary = "Not started";

    private double trialCost;
    private double trialStartPosition;
    private double nextPdTestTarget;
    private double lastTrialSampleSeconds;
    private double lastTrialError;
    private double lastErrorCrossingSeconds;
    private int errorZeroCrossings;
    private double saturatedSeconds;
    private String trialBadReason;

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
        evaluation = null;
        testActive = false;
        completedTestCount = 0;
        operatorCheckSummary = "Pending PD optimization";
        automaticTuneSummary = "Waiting for static-friction measurement";
        csv = TuningCsvWriter.open(
                "pds_" + axis.toString().toLowerCase(),
                "time_s", "axis", "state", "iteration", "evaluation", "repeat",
                "target", "position", "error", "velocity", "command", "kp", "kd", "ks",
                "trial_cost", "error_zero_crossings", "saturation_fraction", "trial_status"
        );
    }

    private void resetAxisPose(TunerContext context) {
        Pose stagingPose = stagingPoseFor(axis);
        if (Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            context.positionRobotForSimulation(stagingPose);
        } else {
            // Re-zeroing odometry does not move the real robot. Alternating trial directions keep
            // the repeated tests near the same physical location.
            context.getFollower().setPose(Pose.zero());
        }
        startValue = getValue(stagingPose);
    }

    /** Bidirectional PDS tests stage at field center. */
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
            case SETTLING_FOR_PD_TEST:
                if (settle(context, PDSState.TUNING_PD)) { beginPdTrial(context); }
                return false;
            case TUNING_PD:
                return updatePdTrial(context);
            case SETTLING_FOR_OPERATOR_CHECK:
                if (settle(context, PDSState.OPERATOR_CHECK)) { beginOperatorCheck(); }
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
        double position = getRelativePosition(context);
        if (!Double.isFinite(position)) {
            abort(context, "Static-friction test produced a non-finite position");
        }
        double movement = Math.abs(position);
        logSample(context, 0.0, position, command);

        boolean moved = movement > threshold;
        if (!moved && timer.milliseconds() < GUESS_TIME_MS) { return false; }

        boolean keepTuning = search.updateGuess(!moved);
        if (keepTuning) {
            state = PDSState.SETTLING_BETWEEN_KS;
        } else {
            controller.getCoefficients().setkS(search.getGuess());
            initializePdTuning(context);
            state = PDSState.SETTLING_FOR_PD_TEST;
        }
        timer.reset();
        return false;
    }

    /** Returns true on the loop where the requested next state is entered. */
    private boolean settle(TunerContext context, PDSState nextState) {
        context.getFollower().stop();
        logSample(context, 0.0, getRelativePosition(context), 0.0);
        double requiredMs = nextState == PDSState.TUNING_PD
                ? PD_SETTLING_TIME_MS : SETTLING_TIME_MS;
        if (timer.milliseconds() < requiredMs) { return false; }

        resetAxisPose(context);
        state = nextState;
        timer.reset();
        return true;
    }

    private void initializePdTuning(TunerContext context) {
        validateTuningConfiguration();
        PDSCoefficients saved = axis == Axis.HEADING
                ? context.constants.angularCoeffs : context.constants.translationalCoeffs;
        currentKp = startingGain(saved.kP, initialKp(), minKp(), maxKp());
        currentKd = startingGain(saved.kD, initialKd(), minKd(), maxKd());
        bestKp = currentKp;
        bestKd = currentKd;
        bestCost = Double.POSITIVE_INFINITY;
        learningRate = Range.clip(PD_INITIAL_LEARNING_RATE,
                PD_MIN_LEARNING_RATE, PD_MAX_LEARNING_RATE);
        iteration = 0;
        smallGradientIterations = 0;
        smallImprovementIterations = 0;
        pdTuningStartedSeconds = sessionTimer.seconds();
        automaticTuneSummary = "Starting repeated closed-loop tests";
        finiteDifferenceTrialsSafe = true;
        nextPdTestTarget = trialMagnitudeFor(axis);
        selectEvaluation(Evaluation.BASELINE);
    }

    private void validateTuningConfiguration() {
        boolean validBounds = Double.isFinite(minKp()) && Double.isFinite(maxKp()) &&
                maxKp() > minKp() && Double.isFinite(minKd()) && Double.isFinite(maxKd()) &&
                maxKd() > minKd();
        boolean validOptimizer = PD_TRIAL_REPEATS > 0 && PD_MAX_ITERATIONS > 0 &&
                PD_CONVERGENCE_PATIENCE > 0 &&
                Double.isFinite(PD_FINITE_DIFFERENCE_FRACTION) &&
                PD_FINITE_DIFFERENCE_FRACTION > 0.0 &&
                PD_FINITE_DIFFERENCE_FRACTION < 0.5 &&
                Double.isFinite(PD_MIN_LEARNING_RATE) && PD_MIN_LEARNING_RATE > 0.0 &&
                Double.isFinite(PD_MAX_LEARNING_RATE) &&
                PD_MAX_LEARNING_RATE >= PD_MIN_LEARNING_RATE &&
                Double.isFinite(PD_INITIAL_LEARNING_RATE) &&
                Double.isFinite(PD_RELATIVE_IMPROVEMENT_TOLERANCE) &&
                PD_RELATIVE_IMPROVEMENT_TOLERANCE >= 0.0 &&
                Double.isFinite(PD_NORMALIZED_GRADIENT_TOLERANCE) &&
                PD_NORMALIZED_GRADIENT_TOLERANCE >= 0.0 &&
                Double.isFinite(initialKp()) && Double.isFinite(initialKd()) &&
                Double.isFinite(PD_MAX_TUNING_SECONDS) && PD_MAX_TUNING_SECONDS > 0.0;
        if (!validBounds || !validOptimizer) {
            throw new IllegalStateException("Invalid PDS auto-tuning bounds or optimizer settings");
        }
    }

    private void selectEvaluation(Evaluation next) {
        evaluation = next;
        trialRepeat = 0;
        evaluationCostSum = 0.0;
        evaluationBadTrials = 0;

        double pStep = (maxKp() - minKp()) * PD_FINITE_DIFFERENCE_FRACTION;
        double dStep = (maxKd() - minKd()) * PD_FINITE_DIFFERENCE_FRACTION;
        switch (evaluation) {
            case KP_PLUS:
                evaluationKp = perturbGain(currentKp, pStep, minKp(), maxKp(), 1);
                evaluationKd = currentKd;
                break;
            case KP_MINUS:
                evaluationKp = perturbGain(currentKp, pStep, minKp(), maxKp(), -1);
                evaluationKd = currentKd;
                break;
            case KD_PLUS:
                evaluationKp = currentKp;
                evaluationKd = perturbGain(currentKd, dStep, minKd(), maxKd(), 1);
                break;
            case KD_MINUS:
                evaluationKp = currentKp;
                evaluationKd = perturbGain(currentKd, dStep, minKd(), maxKd(), -1);
                break;
            case CANDIDATE:
                evaluationKp = candidateKp;
                evaluationKd = candidateKd;
                break;
            case BASELINE:
            default:
                evaluationKp = currentKp;
                evaluationKd = currentKd;
                break;
        }
    }

    private void beginPdTrial(TunerContext context) {
        setControllerGains(evaluationKp, evaluationKd);
        double magnitude = trialMagnitudeFor(axis);
        trialStartPosition = getRelativePosition(context);
        testTarget = nextPdTestTarget;
        nextPdTestTarget = alternatingTrialTarget(testTarget, magnitude);
        testSettledSince = -1.0;
        testFinalError = testTarget;
        trialCost = 0.0;
        lastTrialSampleSeconds = Double.NaN;
        lastTrialError = Double.NaN;
        lastErrorCrossingSeconds = Double.NEGATIVE_INFINITY;
        errorZeroCrossings = 0;
        saturatedSeconds = 0.0;
        trialBadReason = null;
        controller.reset();
        timer.reset();
    }

    /** Each automatically tuned move starts from a re-zeroed pose, so alternate its direction. */
    static double alternatingTrialTarget(double currentTarget, double magnitude) {
        return currentTarget >= 0.0 ? -Math.abs(magnitude) : Math.abs(magnitude);
    }

    private boolean updatePdTrial(TunerContext context) {
        if (sessionTimer.seconds() - pdTuningStartedSeconds >= PD_MAX_TUNING_SECONDS) {
            finishAutomaticTuning(context, "Stopped at the automatic-tuning time limit");
            return false;
        }

        double elapsed = timer.seconds();
        double position = getRelativePosition(context);
        double velocity = getAxisVelocity(context);
        double error = testTarget - position;
        double dt = Double.isFinite(lastTrialSampleSeconds)
                ? elapsed - lastTrialSampleSeconds : 0.0;

        if (!Double.isFinite(elapsed) || !Double.isFinite(position) ||
                !Double.isFinite(velocity) || !Double.isFinite(error)) {
            return rejectPdTrial(context, "non-finite position, velocity, or error");
        }
        if (dt < 0.0 || dt > MAX_PD_SAMPLE_GAP_SECONDS) {
            return rejectPdTrial(context, "control-loop sample gap exceeded " +
                    MAX_PD_SAMPLE_GAP_SECONDS + " s");
        }

        double errorTolerance = errorTolerance();
        double velocityTolerance = velocityTolerance();
        double rawCommand = ensureTestBreakawayPower(
                controller.calculate(error), error, velocity,
                controller.getCoefficients().kS, errorTolerance, velocityTolerance);
        if (!Double.isFinite(rawCommand)) {
            return rejectPdTrial(context, "controller produced non-finite output");
        }
        double command = Range.clip(rawCommand, -MAX_TEST_POWER, MAX_TEST_POWER);

        double safetyLimit = safetyLimit();
        double trialMovement = Math.abs(testTarget - trialStartPosition);
        if (Math.abs(position) > safetyLimit ||
                Math.abs(error) > Math.max(trialMovement * 2.25,
                        errorTolerance * 4.0)) {
            return rejectPdTrial(context, "test exceeded its bounded travel envelope");
        }

        if (dt > 0.0) {
            trialCost = accumulateTimeWeightedSquaredError(
                    trialCost, elapsed, error, dt);
            if (Math.abs(rawCommand) >= MAX_TEST_POWER) { saturatedSeconds += dt; }
        }
        updateOscillationCount(elapsed, error, errorTolerance);
        lastTrialSampleSeconds = elapsed;
        lastTrialError = error;
        testFinalError = error;

        double saturationFraction = saturationFraction(elapsed);
        if (errorZeroCrossings > MAX_ERROR_ZERO_CROSSINGS) {
            return rejectPdTrial(context, "unstable oscillation detected");
        }
        // A large point move can legitimately spend its first second at the configured test-power
        // ceiling. Treat saturation as unsafe only when the robot has also made very little
        // progress; otherwise the timeout and bounded-travel checks remain the relevant guards.
        if (elapsed >= 1.0 && saturationFraction > MAX_SATURATION_FRACTION &&
                Math.abs(error) > Math.max(errorTolerance, trialMovement * 0.75)) {
            return rejectPdTrial(context, "controller remained saturated");
        }

        move(context, command);
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

        if (!settled) {
            // Approximate another half-second of late residual error. This preserves the smooth
            // time-weighted error objective while making a timed-out endpoint clearly expensive.
            trialCost += elapsed * error * error * TEST_SETTLED_SECONDS;
            if (Math.abs(error) > Math.max(errorTolerance * 4.0,
                    trialMovement * 0.75)) {
                trialBadReason = "timed out with obviously large residual error";
            }
        }
        if (saturationFraction > 0.0) {
            trialCost *= 1.0 + 0.25 * saturationFraction;
        }
        return completePdTrial(context);
    }

    private void updateOscillationCount(double elapsed, double error, double tolerance) {
        if (!Double.isFinite(lastTrialError)) { return; }
        boolean crossed = Math.signum(error) != Math.signum(lastTrialError);
        boolean meaningful = Math.max(Math.abs(error), Math.abs(lastTrialError)) >
                tolerance * 0.25;
        if (crossed && meaningful && elapsed - lastErrorCrossingSeconds >= 0.08) {
            errorZeroCrossings++;
            lastErrorCrossingSeconds = elapsed;
        }
    }

    private boolean rejectPdTrial(TunerContext context, String reason) {
        trialBadReason = reason;
        trialCost = badTrialCost(testTarget - trialStartPosition);
        return completePdTrial(context);
    }

    private boolean completePdTrial(TunerContext context) {
        context.getFollower().stop();
        if (!Double.isFinite(trialCost) || trialCost < 0.0) {
            trialBadReason = "non-finite trial cost";
            trialCost = badTrialCost(testTarget - trialStartPosition);
        }
        evaluationCostSum += trialCost;
        if (trialBadReason != null) { evaluationBadTrials++; }
        trialRepeat++;

        if (trialRepeat < PD_TRIAL_REPEATS) {
            state = PDSState.SETTLING_FOR_PD_TEST;
            timer.reset();
            return false;
        }

        double averageCost = evaluationCostSum / PD_TRIAL_REPEATS;
        if (evaluationBadTrials == 0) {
            considerBest(evaluationKp, evaluationKd, averageCost);
        }
        advanceEvaluation(context, averageCost);
        return false;
    }

    private void advanceEvaluation(TunerContext context, double averageCost) {
        switch (evaluation) {
            case BASELINE:
                baselineCost = averageCost;
                finiteDifferenceTrialsSafe &= evaluationBadTrials == 0;
                if (evaluationBadTrials > 0) {
                    if (!Double.isFinite(bestCost)) {
                        abort(context, "Initial closed-loop PD trials were unsafe");
                    }
                    finishRejectedIteration(context, "Unsafe baseline trial");
                    return;
                }
                selectEvaluation(Evaluation.KP_PLUS);
                break;
            case KP_PLUS:
                kpPlusCost = averageCost;
                finiteDifferenceTrialsSafe &= evaluationBadTrials == 0;
                selectEvaluation(Evaluation.KP_MINUS);
                break;
            case KP_MINUS:
                kpMinusCost = averageCost;
                finiteDifferenceTrialsSafe &= evaluationBadTrials == 0;
                selectEvaluation(Evaluation.KD_PLUS);
                break;
            case KD_PLUS:
                kdPlusCost = averageCost;
                finiteDifferenceTrialsSafe &= evaluationBadTrials == 0;
                selectEvaluation(Evaluation.KD_MINUS);
                break;
            case KD_MINUS:
                kdMinusCost = averageCost;
                finiteDifferenceTrialsSafe &= evaluationBadTrials == 0;
                if (!prepareCandidate()) {
                    finishRejectedIteration(context, "Unsafe finite-difference trial");
                    return;
                }
                selectEvaluation(Evaluation.CANDIDATE);
                break;
            case CANDIDATE:
                finishCandidateIteration(context, averageCost);
                return;
            default:
                throw new IllegalStateException("Unknown PD evaluation");
        }
        state = PDSState.SETTLING_FOR_PD_TEST;
        timer.reset();
    }

    private boolean prepareCandidate() {
        if (!finiteDifferenceTrialsSafe || !Double.isFinite(baselineCost) ||
                baselineCost <= 0.0) {
            return false;
        }
        double pPlus = perturbGain(currentKp,
                (maxKp() - minKp()) * PD_FINITE_DIFFERENCE_FRACTION,
                minKp(), maxKp(), 1);
        double pMinus = perturbGain(currentKp,
                (maxKp() - minKp()) * PD_FINITE_DIFFERENCE_FRACTION,
                minKp(), maxKp(), -1);
        double dPlus = perturbGain(currentKd,
                (maxKd() - minKd()) * PD_FINITE_DIFFERENCE_FRACTION,
                minKd(), maxKd(), 1);
        double dMinus = perturbGain(currentKd,
                (maxKd() - minKd()) * PD_FINITE_DIFFERENCE_FRACTION,
                minKd(), maxKd(), -1);

        double costScale = Math.max(baselineCost, 1e-12);
        normalizedGradientP = normalizedFiniteDifference(
                kpPlusCost, kpMinusCost, pPlus, pMinus, minKp(), maxKp(), costScale);
        normalizedGradientD = normalizedFiniteDifference(
                kdPlusCost, kdMinusCost, dPlus, dMinus, minKd(), maxKd(), costScale);
        normalizedGradientMagnitude = Math.hypot(normalizedGradientP, normalizedGradientD);
        if (!Double.isFinite(normalizedGradientMagnitude)) { return false; }

        if (normalizedGradientMagnitude <= PD_NORMALIZED_GRADIENT_TOLERANCE) {
            smallGradientIterations++;
        } else {
            smallGradientIterations = 0;
        }

        candidateKp = gradientUpdatedGain(currentKp, normalizedGradientP,
                normalizedGradientMagnitude, learningRate, minKp(), maxKp());
        candidateKd = gradientUpdatedGain(currentKd, normalizedGradientD,
                normalizedGradientMagnitude, learningRate, minKd(), maxKd());
        return Double.isFinite(candidateKp) && Double.isFinite(candidateKd);
    }

    private void finishCandidateIteration(TunerContext context, double candidateCost) {
        double relativeImprovement = relativeImprovement(baselineCost, candidateCost);
        boolean candidateSafe = evaluationBadTrials == 0 && Double.isFinite(candidateCost);
        // Repeated robot trials contain enough noise that accepting every numerically smaller
        // score makes gains bounce around a flat minimum. Require a meaningful improvement.
        boolean improved = candidateSafe &&
                relativeImprovement >= PD_RELATIVE_IMPROVEMENT_TOLERANCE;
        if (improved) {
            currentKp = candidateKp;
            currentKd = candidateKd;
            setControllerGains(currentKp, currentKd);
            smallImprovementIterations = 0;
            learningRate = Math.min(PD_MAX_LEARNING_RATE, learningRate * 1.05);
            automaticTuneSummary = "Accepted gradient update; improvement " +
                    percent(relativeImprovement);
        } else {
            setControllerGains(currentKp, currentKd);
            learningRate = reducedLearningRate(learningRate, PD_MIN_LEARNING_RATE);
            smallImprovementIterations++;
            automaticTuneSummary = candidateSafe
                    ? "Rejected worse update; reduced learning rate"
                    : "Rejected unsafe update; reduced learning rate";
        }

        iteration++;
        if (shouldStopTuning()) {
            finishAutomaticTuning(context, stopReason());
            return;
        }
        resetIterationCosts();
        selectEvaluation(Evaluation.BASELINE);
        state = PDSState.SETTLING_FOR_PD_TEST;
        timer.reset();
    }

    private void finishRejectedIteration(TunerContext context, String reason) {
        setControllerGains(currentKp, currentKd);
        learningRate = reducedLearningRate(learningRate, PD_MIN_LEARNING_RATE);
        smallImprovementIterations++;
        smallGradientIterations = 0;
        iteration++;
        automaticTuneSummary = reason + "; reduced learning rate";
        if (shouldStopTuning()) {
            finishAutomaticTuning(context, stopReason());
            return;
        }
        resetIterationCosts();
        selectEvaluation(Evaluation.BASELINE);
        state = PDSState.SETTLING_FOR_PD_TEST;
        timer.reset();
    }

    private void resetIterationCosts() {
        baselineCost = Double.NaN;
        kpPlusCost = Double.NaN;
        kpMinusCost = Double.NaN;
        kdPlusCost = Double.NaN;
        kdMinusCost = Double.NaN;
        normalizedGradientP = Double.NaN;
        normalizedGradientD = Double.NaN;
        normalizedGradientMagnitude = Double.NaN;
        finiteDifferenceTrialsSafe = true;
    }

    private boolean shouldStopTuning() {
        return iteration >= PD_MAX_ITERATIONS ||
                smallGradientIterations >= PD_CONVERGENCE_PATIENCE ||
                smallImprovementIterations >= PD_CONVERGENCE_PATIENCE ||
                learningRate <= PD_MIN_LEARNING_RATE;
    }

    private String stopReason() {
        if (smallGradientIterations >= PD_CONVERGENCE_PATIENCE) {
            return "Gradient remained small for " + smallGradientIterations + " iterations";
        }
        if (smallImprovementIterations >= PD_CONVERGENCE_PATIENCE) {
            return "Improvement remained small for " + smallImprovementIterations + " iterations";
        }
        if (learningRate <= PD_MIN_LEARNING_RATE) {
            return "Learning rate reached its minimum";
        }
        return "Reached the configured iteration limit";
    }

    private void finishAutomaticTuning(TunerContext context, String reason) {
        context.getFollower().stop();
        if (Double.isFinite(bestCost)) {
            setControllerGains(bestKp, bestKd);
        } else {
            setControllerGains(currentKp, currentKd);
        }
        automaticTuneSummary = reason + "; restored best measured gains";
        operatorCheckSummary = automaticTuneSummary;
        evaluation = null;
        state = PDSState.SETTLING_FOR_OPERATOR_CHECK;
        timer.reset();
    }

    private void considerBest(double kP, double kD, double cost) {
        if (!Double.isFinite(cost) || cost >= bestCost) { return; }
        bestCost = cost;
        bestKp = kP;
        bestKd = kD;
    }

    private void setControllerGains(double kP, double kD) {
        controller.getCoefficients().setkP(Range.clip(kP, minKp(), maxKp()));
        controller.getCoefficients().setkD(Range.clip(kD, minKd(), maxKd()));
    }

    private void beginOperatorCheck() {
        nextTestTarget = trialMagnitudeFor(axis);
        testTarget = 0.0;
        testSettledSince = -1.0;
        testFinalError = Double.NaN;
        testActive = false;
        operatorCheckSummary = "Ready for operator check (" + automaticTuneSummary + ")";
        controller.reset();
    }

    private boolean updateOperatorCheck(TunerContext context) {
        if (!testActive) {
            context.getFollower().stop();
            logSample(context, testTarget, getRelativePosition(context), 0.0);

            if (context.testButtonWasPressed()) {
                testTarget = nextTestTarget;
                nextTestTarget = Math.abs(testTarget) < 1e-9
                        ? trialMagnitudeFor(axis) : 0.0;
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
        double rawCommand = ensureTestBreakawayPower(
                controller.calculate(error), error, velocity,
                controller.getCoefficients().kS, errorTolerance(), velocityTolerance());
        if (!Double.isFinite(position) || !Double.isFinite(velocity) ||
                !Double.isFinite(error) || !Double.isFinite(rawCommand) ||
                Math.abs(position) > safetyLimit()) {
            abort(context, "Operator-requested PDS test exceeded its safe limits");
        }
        double command = Range.clip(rawCommand, -MAX_TEST_POWER, MAX_TEST_POWER);

        move(context, command);
        testFinalError = error;
        logSample(context, testTarget, position, command);

        boolean withinTolerance = Math.abs(error) <= errorTolerance() &&
                Math.abs(velocity) <= velocityTolerance();
        if (withinTolerance) {
            if (testSettledSince < 0.0) { testSettledSince = elapsed; }
        } else {
            testSettledSince = -1.0;
        }

        boolean settled = testSettledSince >= 0.0 &&
                elapsed - testSettledSince >= TEST_SETTLED_SECONDS;
        if (!settled && elapsed < OPERATOR_TEST_TIMEOUT_SECONDS) { return false; }

        context.getFollower().stop();
        testActive = false;
        completedTestCount++;
        operatorCheckSummary = settled
                ? "Test complete; inspect the response and run again or accept"
                : "Test stopped after timeout; retune if the response is not acceptable";
        return false;
    }

    /**
     * Gives a stalled controller just enough output to cross the measured static-friction floor.
     * The floor is used only outside the endpoint tolerance and is capped by the test power limit.
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

    static double accumulateTimeWeightedSquaredError(double accumulatedCost,
                                                     double elapsedSeconds,
                                                     double error,
                                                     double deltaTimeSeconds) {
        if (!Double.isFinite(accumulatedCost) || accumulatedCost < 0.0 ||
                !Double.isFinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
                !Double.isFinite(error) || !Double.isFinite(deltaTimeSeconds) ||
                deltaTimeSeconds < 0.0) {
            return Double.NaN;
        }
        return accumulatedCost + elapsedSeconds * error * error * deltaTimeSeconds;
    }

    static double normalizedFiniteDifference(double plusCost, double minusCost,
                                             double plusGain, double minusGain,
                                             double minGain, double maxGain,
                                             double costScale) {
        double normalizedSeparation = (plusGain - minusGain) / (maxGain - minGain);
        if (!Double.isFinite(plusCost) || !Double.isFinite(minusCost) ||
                !Double.isFinite(normalizedSeparation) || normalizedSeparation <= 0.0 ||
                !Double.isFinite(costScale) || costScale <= 0.0) {
            return Double.NaN;
        }
        return (plusCost - minusCost) / normalizedSeparation / costScale;
    }

    static double gradientUpdatedGain(double gain, double normalizedGradient,
                                      double gradientMagnitude, double learningRate,
                                      double minGain, double maxGain) {
        if (!Double.isFinite(gain) || !Double.isFinite(normalizedGradient) ||
                !Double.isFinite(gradientMagnitude) || gradientMagnitude < 0.0 ||
                !Double.isFinite(learningRate) || learningRate < 0.0 ||
                !Double.isFinite(minGain) || !Double.isFinite(maxGain) || maxGain <= minGain) {
            return Double.NaN;
        }
        if (gradientMagnitude == 0.0) { return Range.clip(gain, minGain, maxGain); }
        double normalizedGain = (gain - minGain) / (maxGain - minGain);
        double updated = normalizedGain -
                learningRate * normalizedGradient / gradientMagnitude;
        return Range.clip(minGain + updated * (maxGain - minGain), minGain, maxGain);
    }

    static double perturbGain(double gain, double requestedStep,
                              double minGain, double maxGain, int direction) {
        if (direction != -1 && direction != 1) {
            throw new IllegalArgumentException("Finite-difference direction must be -1 or 1");
        }
        double roomBelow = Math.max(0.0, gain - minGain);
        double roomAbove = Math.max(0.0, maxGain - gain);
        double symmetricStep = Math.min(requestedStep, Math.min(roomBelow, roomAbove));
        double step = symmetricStep > 1e-12 ? symmetricStep : requestedStep;
        return Range.clip(gain + direction * step, minGain, maxGain);
    }

    static double reducedLearningRate(double learningRate, double minimum) {
        return Math.max(minimum, learningRate * 0.5);
    }

    static double relativeImprovement(double oldCost, double newCost) {
        if (!Double.isFinite(oldCost) || oldCost <= 0.0 || !Double.isFinite(newCost)) {
            return 0.0;
        }
        return Math.max(0.0, (oldCost - newCost) / oldCost);
    }

    private double badTrialCost(double target) {
        return 10.0 * TEST_TIMEOUT_SECONDS * target * target;
    }

    private double saturationFraction(double elapsed) {
        return elapsed <= 0.0 ? 0.0 : Range.clip(saturatedSeconds / elapsed, 0.0, 1.0);
    }

    private double minKp() {
        return axis == Axis.HEADING ? HEADING_KP_MIN : TRANSLATIONAL_KP_MIN;
    }

    private double maxKp() {
        return axis == Axis.HEADING ? HEADING_KP_MAX : TRANSLATIONAL_KP_MAX;
    }

    private double minKd() {
        return axis == Axis.HEADING ? HEADING_KD_MIN : TRANSLATIONAL_KD_MIN;
    }

    private double maxKd() {
        return axis == Axis.HEADING ? HEADING_KD_MAX : TRANSLATIONAL_KD_MAX;
    }

    private double initialKp() {
        return axis == Axis.HEADING ? HEADING_INITIAL_KP : TRANSLATIONAL_INITIAL_KP;
    }

    private double initialKd() {
        return axis == Axis.HEADING ? HEADING_INITIAL_KD : TRANSLATIONAL_INITIAL_KD;
    }

    private double errorTolerance() {
        return axis == Axis.HEADING ? Math.toRadians(2.5) : 0.75;
    }

    private double velocityTolerance() {
        return axis == Axis.HEADING ? 0.10 : 1.0;
    }

    private double safetyLimit() {
        return axis == Axis.HEADING ? Math.toRadians(110.0) : 36.0;
    }

    static double trialMagnitudeFor(Axis axis) {
        return axis == Axis.HEADING ? Math.toRadians(60.0) : 24.0;
    }

    private static double startingGain(double saved, double fallback,
                                       double minimum, double maximum) {
        double candidate = Double.isFinite(saved) && saved > minimum ? saved : fallback;
        return Range.clip(candidate, minimum, maximum);
    }

    private void abort(TunerContext context, String reason) {
        context.getFollower().stop();
        operatorCheckSummary = "FAILED: " + reason;
        if (csv != null) { csv.close(); }
        throw new IllegalStateException(reason + ". See PDS CSV: " + getCsvPath());
    }

    private void logSample(TunerContext context, double target, double position, double command) {
        if (csv == null) { return; }
        double elapsed = state == PDSState.TUNING_PD ? timer.seconds() : 0.0;
        csv.writeRow(
                sessionTimer.seconds(), axis, state, iteration,
                evaluation == null ? "" : evaluation,
                evaluation == null ? "" : trialRepeat + 1,
                target, position, target - position, getAxisVelocity(context), command,
                controller.getCoefficients().kP, controller.getCoefficients().kD,
                controller.getCoefficients().kS, trialCost, errorZeroCrossings,
                saturationFraction(elapsed), trialBadReason == null ? "OK" : trialBadReason
        );
    }

    private static String percent(double fraction) {
        return Math.round(fraction * 1000.0) / 10.0 + "%";
    }

    void reportProgress(TunerContext context) {
        context.getTelemetry().addLine("Automatic " + axis.toString().toLowerCase() +
                " tuning in progress");
        context.getTelemetry().addLine(actionDescription(context));
        if (state == PDSState.TUNING_PD || state == PDSState.OPERATOR_CHECK) {
            double position = getRelativePosition(context);
            if (axis == Axis.HEADING) {
                context.getTelemetry().addData("Target", round(Math.toDegrees(testTarget), 2) +
                        " deg");
                context.getTelemetry().addData("Position", round(Math.toDegrees(position), 2) +
                        " deg");
            } else {
                context.getTelemetry().addData("Target", round(testTarget, 2) + " in");
                context.getTelemetry().addData("Position", round(position, 2) + " in");
            }
        }
        if (!context.isDebugMode()) {
            context.getTelemetry().update();
            return;
        }
        context.getTelemetry().addData("Step", state.toString().replace('_', ' '));
        context.getTelemetry().addData("Static guess", search.getGuess());
        if (evaluation != null) {
            context.getTelemetry().addData("PD iteration", (iteration + 1) + " / " +
                    PD_MAX_ITERATIONS);
            context.getTelemetry().addData("Evaluation", evaluation + " repeat " +
                    (trialRepeat + 1) + " / " + PD_TRIAL_REPEATS);
            context.getTelemetry().addData("Evaluation gains",
                    "kP=" + evaluationKp + ", kD=" + evaluationKd);
            context.getTelemetry().addData("Current cost", trialCost);
            context.getTelemetry().addData("Learning rate", learningRate);
            context.getTelemetry().addData("Normalized gradient",
                    "kP=" + normalizedGradientP + ", kD=" + normalizedGradientD);
            context.getTelemetry().addData("Best measured",
                    "J=" + bestCost + ", kP=" + bestKp + ", kD=" + bestKd);
            context.getTelemetry().addData("Trial safeguards",
                    trialBadReason == null ? "OK" : trialBadReason);
        }
        if (state == PDSState.OPERATOR_CHECK) {
            context.getTelemetry().addData("Operator tests completed", completedTestCount);
            context.getTelemetry().addData("Current test error", testFinalError);
        }
        context.getTelemetry().addData("CSV", getCsvPath());
        context.getTelemetry().addLine(state == PDSState.OPERATOR_CHECK
                ? "The robot remains stopped until you request or accept a test."
                : "Keep the OpMode running until automatic tuning finishes.");
        context.getTelemetry().update();
    }

    private static double round(double value, int decimalPlaces) {
        double scale = Math.pow(10.0, decimalPlaces);
        return Math.round(value * scale) / scale;
    }

    private String actionDescription(TunerContext context) {
        switch (state) {
            case TUNING_KS:
                return axis == Axis.HEADING
                        ? "Robot is finding the minimum power needed to turn."
                        : "Robot is finding the minimum power needed to move.";
            case SETTLING_BETWEEN_KS:
            case SETTLING_FOR_PD_TEST:
            case SETTLING_FOR_OPERATOR_CHECK:
                return "Robot is stopping before the next test.";
            case TUNING_PD:
                return axis == Axis.HEADING
                        ? "Robot is running repeated closed-loop turns to tune kP and kD."
                        : "Robot is running repeated closed-loop moves to tune kP and kD.";
            case OPERATOR_CHECK:
                if (testActive) {
                    return axis == Axis.HEADING
                            ? "Robot is turning to the requested test target."
                            : "Robot is driving to the requested test target.";
                }
                return operatorCheckSummary +
                        ". Press X to run the next alternating test, A to accept, or B to retune.";
            default:
                return "Robot is running the controller test.";
        }
    }

    PDSCoefficients getCoefficients() { return controller.getCoefficients(); }

    String getOperatorCheckSummary() { return operatorCheckSummary; }

    String getCsvPath() { return csv == null ? "Unavailable" : csv.getPath(); }
}
