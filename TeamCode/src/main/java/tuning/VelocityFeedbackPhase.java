package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import feedforward.MotionParameters;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Tunes the velocity feedback gains for the follower by running a forward and backward path/turn
 * and measuring the RMS error between the desired and actual velocities. The user can also manually
 * tune the gains.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class VelocityFeedbackPhase extends TuningPhase {
    private static final int SEARCH_ROUNDS = 4;
    private static final double DIRECTION_TIMEOUT_SECONDS = 12.0;
    private static final double SAMPLE_EDGE_FRACTION = 0.10;
    /** Prevents angular velocity feedback from becoming a noisy bang-bang controller. */
    public static double MAX_ANGULAR_FEEDBACK_GAIN = 0.25;

    enum FeedbackAxis { TRANSLATION, ANGULAR }

    private final double[] gains = new double[3];
    private final double[] scores = new double[3];
    private final CandidateResult[] candidateResults = new CandidateResult[3];

    private Path forwardPath, backwardPath;
    private Turn forwardTurn, backwardTurn;

    private FollowerMovement currentMovement;
    private boolean forwardIsRunning;

    private FeedbackAxis axis;
    private int candidate;
    private double center;
    private double step;
    private int round;

    private double errorSquared;
    private int errorSamples;
    private final double[] directionErrorSquared = new double[2];
    private final int[] directionErrorSamples = new int[2];
    private final int[] directionSaturatedSamples = new int[2];
    private final int[] directionTotalSamples = new int[2];
    private final double[] directionCompletionSeconds = new double[2];
    private final double[] directionFinalError = new double[2];
    private final double[] directionOvershoot = new double[2];
    private double lastScore;
    private double translationScore;
    private double angularScore;
    private double bestTranslationScore = Double.POSITIVE_INFINITY;
    private double bestAngularScore = Double.POSITIVE_INFINITY;
    private boolean manualTestRunning;
    private int manualTestNumber;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private TuningCsvWriter responseCsv;
    private String responseCsvPath = "Not started";
    private double incumbentGain;
    private CandidateResult incumbentResult;
    private String acceptanceMessage = "Pending";
    private final ElapsedTime directionTimer = new ElapsedTime();
    private final ElapsedTime trialTimer = new ElapsedTime();

    public VelocityFeedbackPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Velocity Feedback"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Translation needs a clear 48-inch out-and-back lane.");
        context.getTelemetry().addLine(
                "Angular feedback needs room for a 90-degree out-and-back turn.");
        context.getTelemetry().addLine(
                "Manual tests remain stopped until X is pressed.");
    }

    @Override
    protected void init() {
        if (context.constants.velocityFeedbackGain <= 0.0 &&
                context.constants.translationalCoeffs.kD > 0.0) {
            context.constants.velocityFeedbackGain = context.constants.translationalCoeffs.kD;
        }
        if (context.constants.angularVelocityFeedbackGain <= 0.0 &&
                context.constants.angularCoeffs.kD > 0.0) {
            context.constants.angularVelocityFeedbackGain = context.constants.angularCoeffs.kD;
        }
        context.constants.angularVelocityFeedbackGain = Math.min(
                context.constants.angularVelocityFeedbackGain, MAX_ANGULAR_FEEDBACK_GAIN);
        applyCurrentGains();

        GeometryFactory factory = new GeometryFactory(context.getFollower())
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);

        // Center the complete 48-inch translation footprint on the field.
        Pose start = factory.pose(-24, 0, 0);
        Pose end = factory.pose(24, 0, 0);
        context.getFollower().setPose(start);
        forwardPath = factory.path(start, end)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).profiledBuild();
        backwardPath = factory.path(end, start)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).profiledBuild();

        Pose turned = factory.pose(-24, 0, 90);
        forwardTurn = factory.turn(start).turnTo(turned.getHeading()).profiledBuild();
        backwardTurn = factory.turn(turned).turnTo(start.getHeading()).profiledBuild();

        axis = FeedbackAxis.TRANSLATION;
        if (manualMode) {
            context.getFollower().stop();
            manualTestRunning = false;
            manualTestNumber = 0;
            manualCsv = openResponseCsv("manual_velocity_feedback");
            responseCsv = manualCsv;
            manualCsvPath = manualCsv.getPath();
            responseCsvPath = manualCsvPath;
        } else {
            responseCsv = openResponseCsv("automatic_velocity_feedback");
            responseCsvPath = responseCsv.getPath();
            startSearch(FeedbackAxis.TRANSLATION);
        }
    }

    private TuningCsvWriter openResponseCsv(String name) {
        return TuningCsvWriter.open(name,
                "test", "timestamp_s", "axis", "direction", "gain",
                "target_velocity", "raw_velocity", "kalman_velocity",
                "command_power", "displacement", "heading_error", "saturated",
                "velocity_error", "sample_region");
    }

    private void applyCurrentGains() {
        context.getFollower().setVelocityFeedback(
                context.constants.velocityFeedbackGain,
                context.constants.angularVelocityFeedbackGain
        );
    }

    private void startSearch(FeedbackAxis nextAxis) {
        axis = nextAxis;
        incumbentGain = (axis == FeedbackAxis.TRANSLATION) ?
                context.constants.velocityFeedbackGain :
                Math.min(context.constants.angularVelocityFeedbackGain,
                        MAX_ANGULAR_FEEDBACK_GAIN);
        center = incumbentGain;
        incumbentResult = null;
        double feedforward = (axis == FeedbackAxis.TRANSLATION) ?
                context.constants.translationalKV : context.constants.angularKV;

        step = Math.max(center * 0.5, Math.max(feedforward * 0.25, 0.00001));
        if (center <= 0.0) { center = step; }

        round = 0;
        startRound();
    }

    private void startRound() {
        if (round == 0) {
            // Always establish a zero-gain control and repeat the incumbent before exploring.
            gains[0] = 0.0;
            gains[1] = incumbentGain;
            gains[2] = incumbentGain + step;
        } else {
            gains[0] = Math.max(0.0, center - step);
            gains[1] = center;
            gains[2] = center + step;
        }
        if (axis == FeedbackAxis.ANGULAR) {
            for (int i = 0; i < gains.length; i++) {
                gains[i] = Math.min(gains[i], MAX_ANGULAR_FEEDBACK_GAIN);
            }
        }
        candidate = 0;
        startCandidate();
    }

    private void startCandidate() {
        if (axis == FeedbackAxis.TRANSLATION) {
            context.constants.velocityFeedbackGain = gains[candidate];
        } else {
            context.constants.angularVelocityFeedbackGain = gains[candidate];
        }
        applyCurrentGains();
        startTest();
    }

    private void startTest() {
        forwardIsRunning = true;
        errorSquared = 0.0;
        errorSamples = 0;
        for (int i = 0; i < 2; i++) {
            directionErrorSquared[i] = 0.0;
            directionErrorSamples[i] = 0;
            directionSaturatedSamples[i] = 0;
            directionTotalSamples[i] = 0;
            directionCompletionSeconds[i] = 0.0;
            directionFinalError[i] = 0.0;
            directionOvershoot[i] = 0.0;
        }

        if (axis == FeedbackAxis.TRANSLATION) {
            currentMovement = forwardPath;
        } else {
            currentMovement = forwardTurn;
        }

        context.getFollower().follow(currentMovement);
        directionTimer.reset();
        trialTimer.reset();
    }

    private void sampleTest() {
        if (!context.getFollower().isBusy()) { return; }

        double targetVelocity;
        double rawVelocity;
        double kalmanVelocity;
        double displacement;
        double headingError;
        boolean centralSample;

        if (axis == FeedbackAxis.TRANSLATION) {
            Path path = (Path) currentMovement;
            PathSegment segment = path.getParametricPath();

            // Retrieve pre-calculated 't' directly from Follower to save cycles
            double t = context.getFollower().getBestT();
            Vector target = segment.getPosition(t);
            double remaining = segment.getDistanceToEndIn(target, t);
            double traveled = segment.getLengthIn() - remaining;

            MotionParameters desired = path.getFeedforwardLut().getFFParams(traveled);
            Vector tangent = segment.getFirstDerivative(t).normalize();

            // X is forward, Y is sideways layout works perfectly with this dot product
            targetVelocity = desired.getTangentialVel();
            rawVelocity = context.getFollower().getRawVelocity().getVec().dot(tangent).getIn();
            kalmanVelocity = context.getFollower().getVelocity().getVec().dot(tangent).getIn();
            displacement = traveled;
            headingError = context.getFollower().getPose().getHeading().getShortestAngleTo(
                    path.getEndPose().getHeading()).getRad();
            centralSample = isUsableTranslationSample(
                    targetVelocity, traveled, segment.getLengthIn());
        } else {
            Turn turn = (Turn) currentMovement;
            double traveled = turnProfileProgress(
                    turn, context.getFollower().getPose().getHeading());

            MotionParameters desired = turn.getFeedforwardLut()
                    .getFFParams(traveled);
            targetVelocity = desired.getAngularVel();
            rawVelocity = context.getFollower().getRawVelocity().getHeading().getRad();
            kalmanVelocity = context.getFollower().getVelocity().getHeading().getRad();
            displacement = traveled;
            headingError = context.getFollower().getPose().getHeading().getShortestAngleTo(
                    turn.getEndPose().getHeading()).getRad();
            centralSample = Math.abs(targetVelocity) > 0.05
                    && traveled >= Math.abs(turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad())
                    * SAMPLE_EDGE_FRACTION
                    && traveled <= Math.abs(turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad())
                    * (1.0 - SAMPLE_EDGE_FRACTION);

            double signedSweep = turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
            double signedTravel = turn.getStartPose().getHeading()
                    .getShortestAngleTo(context.getFollower().getPose().getHeading()).getRad()
                    * Math.signum(signedSweep);
            int directionIndex = forwardIsRunning ? 0 : 1;
            directionOvershoot[directionIndex] = Math.max(directionOvershoot[directionIndex],
                    Math.max(0.0, signedTravel - Math.abs(signedSweep)));
        }


        double commandPower = currentCommandPower();
        boolean saturated = commandPower >= 0.98;
        int directionIndex = forwardIsRunning ? 0 : 1;
        directionTotalSamples[directionIndex]++;
        if (saturated) { directionSaturatedSamples[directionIndex]++; }
        if (centralSample) {
            addError(targetVelocity, kalmanVelocity, directionIndex);
        }
        logResponseSample(targetVelocity, rawVelocity, kalmanVelocity, commandPower,
                displacement, headingError, saturated, centralSample);
    }

    private void logResponseSample(double target, double raw, double kalman,
                                   double commandPower, double displacement,
                                   double headingError, boolean saturated,
                                   boolean centralSample) {
        if (responseCsv == null || (manualMode && !manualTestRunning)) { return; }
        double gain = axis == FeedbackAxis.TRANSLATION
                ? context.constants.velocityFeedbackGain
                : context.constants.angularVelocityFeedbackGain;
        responseCsv.writeRow(manualMode ? manualTestNumber : round * gains.length + candidate + 1,
                trialTimer.seconds(), axis,
                forwardIsRunning ? "OUTBOUND" : "RETURN", gain,
                target, raw, kalman, commandPower, displacement, headingError, saturated,
                target - kalman, centralSample ? "CENTRAL_PROFILE" : "ENDPOINT_SETTLING");
    }

    private void addError(double target, double actual, int directionIndex) {
        if (Double.isFinite(target) && Double.isFinite(actual)) {
            double error = target - actual;
            errorSquared += error * error;
            errorSamples++;
            directionErrorSquared[directionIndex] += error * error;
            directionErrorSamples[directionIndex]++;
        }
    }

    private double currentCommandPower() {
        return Math.max(Math.max(
                        Math.abs(context.getFollower().getDrivetrain().getLastFlPower()),
                        Math.abs(context.getFollower().getDrivetrain().getLastFrPower())),
                Math.max(
                        Math.abs(context.getFollower().getDrivetrain().getLastBlPower()),
                        Math.abs(context.getFollower().getDrivetrain().getLastBrPower())));
    }

    private boolean updateTest() {
        sampleTest();

        if (context.getFollower().isBusy()) {
            if (directionTimer.seconds() <= DIRECTION_TIMEOUT_SECONDS) { return false; }
            context.getFollower().stop();
            throw new IllegalStateException(
                    "Velocity feedback " + axis.name().toLowerCase() + " candidate " +
                            (candidate + 1) + " timed out during " +
                            (forwardIsRunning ? "outbound" : "return") +
                            " travel at gain " + gains[candidate] + ". Pose=" +
                            context.getFollower().getPose() + ", velocity=" +
                            context.getFollower().getVelocity() +
                            ". Verify localization, feedforward, and PDS constants."
            );
        }

        finishDirectionMetrics(forwardIsRunning ? 0 : 1);

        if (forwardIsRunning) {
            forwardIsRunning = false;

            if (axis == FeedbackAxis.TRANSLATION) {
                currentMovement = backwardPath;
            } else {
                currentMovement = backwardTurn;
            }

            context.getFollower().follow(currentMovement);
            directionTimer.reset();
            return false;
        }

        if (errorSamples == 0) {
            throw new IllegalStateException(
                    "Velocity feedback trial produced no usable " + axis.name().toLowerCase() +
                            " samples. Verify feedforward constants and localization."
            );
        }
        candidateResults[candidate] = CandidateResult.from(
                directionErrorSquared, directionErrorSamples,
                directionFinalError, directionOvershoot,
                directionCompletionSeconds, directionSaturatedSamples,
                directionTotalSamples);
        lastScore = candidateResults[candidate].centralRms;
        if (round == 0 && candidate == 1) {
            incumbentResult = candidateResults[candidate];
        }
        return true;
    }

    private void finishDirectionMetrics(int directionIndex) {
        directionCompletionSeconds[directionIndex] = directionTimer.seconds();
        if (axis == FeedbackAxis.TRANSLATION) {
            Path path = (Path) currentMovement;
            directionFinalError[directionIndex] = context.getFollower().getPose()
                    .distanceTo(path.getEndPose()).getIn();
        } else {
            Turn turn = (Turn) currentMovement;
            directionFinalError[directionIndex] = Math.abs(context.getFollower().getPose()
                    .getHeading().getShortestAngleTo(turn.getEndPose().getHeading()).getRad());
        }
    }

    static double turnProfileProgress(Turn turn, Angle currentHeading) {
        double signedSweep = turn.getStartPose().getHeading()
                .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
        double direction = Math.signum(signedSweep);
        double signedTravel = turn.getStartPose().getHeading()
                .getShortestAngleTo(currentHeading).getRad();
        return Math.max(0.0, Math.min(Math.abs(signedSweep), signedTravel * direction));
    }

    @Override
    protected boolean autoTuned() {
        reportAutomaticProgress();
        if (!updateTest()) { return false; }

        scores[candidate] = candidateResults[candidate].outboundRms;
        candidate++;
        if (candidate < gains.length) {
            startCandidate();
            return false;
        }

        int best = 0;
        for (int i = 1; i < scores.length; i++) {
            if (scores[i] < scores[best]) { best = i; }
        }

        center = gains[best];
        CandidateResult bestResult = candidateResults[best];

        round++;
        if (round < SEARCH_ROUNDS) {
            step *= 0.5;
            startRound();
            return false;
        }

        if (axis == FeedbackAxis.TRANSLATION) {
            boolean accepted = acceptsCandidate(incumbentResult, bestResult);
            context.constants.velocityFeedbackGain = accepted ? center : incumbentGain;
            translationScore = accepted ? bestResult.centralRms : incumbentResult.centralRms;
            acceptanceMessage = "Translation " + (accepted ? "accepted" : "retained incumbent")
                    + acceptanceDetails(incumbentResult, bestResult)
                    + (accepted ? "" : "; run manual validation");
            applyCurrentGains();
            startSearch(FeedbackAxis.ANGULAR);
            return false;
        }

        // If we are here, we have finished tuning both axes
        boolean accepted = acceptsCandidate(incumbentResult, bestResult);
        context.constants.angularVelocityFeedbackGain = accepted ? center : incumbentGain;
        angularScore = accepted ? bestResult.centralRms : incumbentResult.centralRms;
        acceptanceMessage += "; Angular " +
                (accepted ? "accepted" : "retained incumbent")
                + acceptanceDetails(incumbentResult, bestResult)
                + (accepted ? "" : "; run manual validation");
        applyCurrentGains();
        if (responseCsv != null) { responseCsv.close(); }
        return true;
    }

    static boolean acceptsCandidate(CandidateResult incumbent, CandidateResult candidate) {
        if (incumbent == null || candidate == null ||
                !Double.isFinite(incumbent.returnRms) ||
                !Double.isFinite(candidate.returnRms)) {
            return false;
        }
        boolean heldOutImprovement = candidate.returnRms <= incumbent.returnRms * 0.95;
        boolean directionallyConsistent = candidate.directionDifferenceRatio() <= 0.20;
        boolean notSaturationDependent = candidate.saturationRate <=
                Math.min(0.10, Math.max(0.02, incumbent.saturationRate + 0.02));
        return heldOutImprovement && directionallyConsistent && notSaturationDependent;
    }

    private String acceptanceDetails(CandidateResult incumbent, CandidateResult candidate) {
        if (incumbent == null || candidate == null) { return " (insufficient validation)"; }
        return String.format(java.util.Locale.US,
                " (held-out %.4f -> %.4f, direction gap %.1f%%, saturation %.1f%%)",
                incumbent.returnRms, candidate.returnRms,
                candidate.directionDifferenceRatio() * 100.0,
                candidate.saturationRate * 100.0);
    }

    private void reportAutomaticProgress() {
        context.getTelemetry().addLine("Automatic velocity feedback tuning in progress");
        context.getTelemetry().addLine(actionDescription());
        if (!context.isDebugMode()) {
            context.getTelemetry().update();
            return;
        }
        context.getTelemetry().addData("Axis", axis);
        context.getTelemetry().addData("Search round", (round + 1) + " / " + SEARCH_ROUNDS);
        context.getTelemetry().addData("Candidate", (candidate + 1) + " / " + gains.length);
        context.getTelemetry().addData("Candidate gain", gains[candidate]);
        context.getTelemetry().addData("Direction", forwardIsRunning ? "OUTBOUND" : "RETURN");
        context.getTelemetry().addData("Direction elapsed",
                Math.round(directionTimer.seconds() * 10.0) / 10.0 + " / " +
                        DIRECTION_TIMEOUT_SECONDS + " s");
        int axisOffset = axis == FeedbackAxis.TRANSLATION ? 0 : SEARCH_ROUNDS * gains.length;
        context.getTelemetry().addData("Overall candidate trial",
                (axisOffset + round * gains.length + candidate + 1) + " / " +
                        (2 * SEARCH_ROUNDS * gains.length));
        context.getTelemetry().addData("Usable samples", errorSamples);
        context.getTelemetry().addData("Last RMS score", lastScore);
        context.getTelemetry().addData("Response CSV", responseCsvPath);
        context.getTelemetry().update();
    }

    private String actionDescription() {
        if (axis == FeedbackAxis.TRANSLATION) {
            return forwardIsRunning
                    ? "Robot is driving the outbound test path."
                    : "Robot is driving the return test path.";
        }
        return forwardIsRunning
                ? "Robot is turning to the outbound heading."
                : "Robot is turning back to the starting heading.";
    }

    @Override
    protected boolean manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed() || opMode.gamepad1.rightBumperWasPressed()) {
            context.getFollower().stop();
            manualTestRunning = false;
            axis = (axis == FeedbackAxis.TRANSLATION) ?
                    FeedbackAxis.ANGULAR : FeedbackAxis.TRANSLATION;
        }

        double change = manualChange();
        if (change != 0.0) {
            if (axis == FeedbackAxis.TRANSLATION) {
                context.constants.velocityFeedbackGain = Math.max(0.0,
                        context.constants.velocityFeedbackGain + change);
            } else {
                context.constants.angularVelocityFeedbackGain = Math.max(0.0,
                        Math.min(MAX_ANGULAR_FEEDBACK_GAIN,
                                context.constants.angularVelocityFeedbackGain + change));
            }
            applyCurrentGains();
            if (manualTestRunning) { restartManualTest(); }
        } else if (opMode.gamepad1.xWasPressed()) {
            restartManualTest();
        } else if (manualTestRunning && updateTest()) {
            if (axis == FeedbackAxis.TRANSLATION) {
                translationScore = lastScore;
                bestTranslationScore = Math.min(bestTranslationScore, lastScore);
            } else {
                angularScore = lastScore;
                bestAngularScore = Math.min(bestAngularScore, lastScore);
            }
            manualTestRunning = false;
            context.getFollower().stop();
        }

        addTunableValue("Translation feedback", context.constants.velocityFeedbackGain,
                axis == FeedbackAxis.TRANSLATION);
        addTunableValue("Angular feedback", context.constants.angularVelocityFeedbackGain,
                axis == FeedbackAxis.ANGULAR);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Final RMS error",
                number(axis == FeedbackAxis.TRANSLATION ? translationScore : angularScore));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Test state", manualTestRunning
                    ? actionDescription() : "IDLE - press X to run");
            context.getTelemetry().addData("Usable samples", errorSamples);
            context.getTelemetry().addData("Live RMS error", errorSamples == 0
                    ? Double.NaN : Math.sqrt(errorSquared / errorSamples));
            context.getTelemetry().addData("Best translation RMS", bestTranslationScore);
            context.getTelemetry().addData("Best angular RMS", bestAngularScore);
            context.getTelemetry().addData("Response CSV", manualCsvPath);
        }
        context.getTelemetry().addLine("Dpad Up/Down: change value");
        context.getTelemetry().addLine("LB/RB: select value");
        context.getTelemetry().addLine("X: run/restart test");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            manualTestRunning = false;
            if (manualCsv != null) { manualCsv.close(); }
            return true;
        }

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Translation feedback gain",
                number(context.constants.velocityFeedbackGain));
        context.getTelemetry().addData("Angular feedback gain",
                number(context.constants.angularVelocityFeedbackGain));
        context.getTelemetry().addData("Translation root mean square error", number(translationScore));
        context.getTelemetry().addData("Angular root mean square error", number(angularScore));
        context.getTelemetry().addData("Validation", acceptanceMessage);
        context.getTelemetry().addData("Response CSV", responseCsvPath);
    }

    private void restartManualTest() {
        context.getFollower().stop();
        manualTestNumber++;
        startTest();
        manualTestRunning = true;
    }

    static boolean isUsableTranslationSample(double targetVelocity, double traveled,
                                               double pathLength) {
        if (!Double.isFinite(targetVelocity) || !Double.isFinite(traveled) ||
                !Double.isFinite(pathLength) || pathLength <= 0.0) {
            return false;
        }
        double fraction = traveled / pathLength;
        return Math.abs(targetVelocity) > 1.0 && fraction >= SAMPLE_EDGE_FRACTION &&
                fraction <= 1.0 - SAMPLE_EDGE_FRACTION;
    }

    /** Keeps profile tracking separate from endpoint, settling-time, and saturation evidence. */
    static final class CandidateResult {
        final double outboundRms;
        final double returnRms;
        final double centralRms;
        final double endpointError;
        final double overshoot;
        final double completionSeconds;
        final double saturationRate;

        CandidateResult(double outboundRms, double returnRms, double endpointError,
                        double overshoot, double completionSeconds, double saturationRate) {
            this.outboundRms = outboundRms;
            this.returnRms = returnRms;
            this.centralRms = (outboundRms + returnRms) / 2.0;
            this.endpointError = endpointError;
            this.overshoot = overshoot;
            this.completionSeconds = completionSeconds;
            this.saturationRate = saturationRate;
        }

        static CandidateResult from(double[] squaredError, int[] samples,
                                    double[] finalError, double[] overshoot,
                                    double[] completion, int[] saturated, int[] total) {
            double outbound = samples[0] == 0 ? Double.POSITIVE_INFINITY
                    : Math.sqrt(squaredError[0] / samples[0]);
            double returning = samples[1] == 0 ? Double.POSITIVE_INFINITY
                    : Math.sqrt(squaredError[1] / samples[1]);
            int totalSamples = total[0] + total[1];
            return new CandidateResult(outbound, returning,
                    (finalError[0] + finalError[1]) / 2.0,
                    Math.max(overshoot[0], overshoot[1]),
                    completion[0] + completion[1],
                    totalSamples == 0 ? 1.0
                            : (double) (saturated[0] + saturated[1]) / totalSamples);
        }

        double directionDifferenceRatio() {
            double scale = Math.max(Math.abs(outboundRms), Math.abs(returnRms));
            if (!Double.isFinite(scale) || scale <= 1e-12) { return 0.0; }
            return Math.abs(outboundRms - returnRms) / scale;
        }
    }
}
