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
    private double lastScore;
    private double translationScore;
    private double angularScore;
    private double bestTranslationScore = Double.POSITIVE_INFINITY;
    private double bestAngularScore = Double.POSITIVE_INFINITY;
    private boolean manualTestRunning;
    private int manualTestNumber;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private final ElapsedTime directionTimer = new ElapsedTime();

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
            manualCsv = TuningCsvWriter.open("manual_velocity_feedback",
                    "test", "time_s", "axis", "direction", "gain",
                    "target_velocity", "actual_velocity", "error");
            manualCsvPath = manualCsv.getPath();
        } else {
            startSearch(FeedbackAxis.TRANSLATION);
        }
    }

    private void applyCurrentGains() {
        context.getFollower().setVelocityFeedback(
                context.constants.velocityFeedbackGain,
                context.constants.angularVelocityFeedbackGain
        );
    }

    private void startSearch(FeedbackAxis nextAxis) {
        axis = nextAxis;
        center = (axis == FeedbackAxis.TRANSLATION) ? context.constants.velocityFeedbackGain :
                Math.min(context.constants.angularVelocityFeedbackGain,
                        MAX_ANGULAR_FEEDBACK_GAIN);
        double feedforward = (axis == FeedbackAxis.TRANSLATION) ?
                context.constants.translationalKV : context.constants.angularKV;

        step = Math.max(center * 0.5, Math.max(feedforward * 0.25, 0.00001));
        if (center <= 0.0) { center = step; }

        round = 0;
        startRound();
    }

    private void startRound() {
        gains[0] = Math.max(0.0, center - step);
        gains[1] = center;
        gains[2] = center + step;
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

        if (axis == FeedbackAxis.TRANSLATION) {
            currentMovement = forwardPath;
        } else {
            currentMovement = forwardTurn;
        }

        context.getFollower().follow(currentMovement);
        directionTimer.reset();
    }

    private void sampleTest() {
        if (!context.getFollower().isBusy()) { return; }

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
            double actual = context.getFollower().getVelocity().getVec().dot(tangent).getIn();
            if (isUsableTranslationSample(
                    desired.getTangentialVel(), traveled, segment.getLengthIn())) {
                addError(desired.getTangentialVel(), actual, 1.0);
                logManualSample(desired.getTangentialVel(), actual);
            }
        } else {
            Turn turn = (Turn) currentMovement;
            double traveled = turnProfileProgress(
                    turn, context.getFollower().getPose().getHeading());

            MotionParameters desired = turn.getFeedforwardLut()
                    .getFFParams(traveled);
            double actual = context.getFollower().getVelocity().getHeading().getRad();
            addError(desired.getAngularVel(), actual, 0.05);
            logManualSample(desired.getAngularVel(), actual);
        }
    }

    private void logManualSample(double target, double actual) {
        if (!manualMode || !manualTestRunning || manualCsv == null) { return; }
        double gain = axis == FeedbackAxis.TRANSLATION
                ? context.constants.velocityFeedbackGain
                : context.constants.angularVelocityFeedbackGain;
        manualCsv.writeRow(manualTestNumber, directionTimer.seconds(), axis,
                forwardIsRunning ? "OUTBOUND" : "RETURN", gain,
                target, actual, target - actual);
    }

    private void addError(double target, double actual, double minimumTarget) {
        if (Double.isFinite(target) && Double.isFinite(actual) &&
                Math.abs(target) > minimumTarget) {
            double error = target - actual;
            errorSquared += error * error;
            errorSamples++;
        }
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
        lastScore = Math.sqrt(errorSquared / errorSamples);
        return true;
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

        scores[candidate] = lastScore;
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

        round++;
        if (round < SEARCH_ROUNDS) {
            step *= 0.5;
            startRound();
            return false;
        }

        if (axis == FeedbackAxis.TRANSLATION) {
            translationScore = scores[best];
            context.constants.velocityFeedbackGain = center;
            applyCurrentGains();
            startSearch(FeedbackAxis.ANGULAR);
            return false;
        }

        // If we are here, we have finished tuning both axes
        angularScore = scores[best];
        context.constants.angularVelocityFeedbackGain = center;
        return true;
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
}
