package tuning;

import feedforward.MotionParameters;
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

    enum FeedbackAxis { TRANSLATION, ANGULAR }

    private final double[] gains = new double[3];
    private final double[] scores = new double[3];

    private Path forwardPath, backwardPath;
    private Turn forwardTurn, backwardTurn;

    private FollowerMovement currentMovement;
    private boolean forwardIsRunning;
    private double currentTurnDirection;

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

    public VelocityFeedbackPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Velocity Feedback"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        if (context.constants.translationalCoeffs.kD > 0.0) {
            context.constants.velocityFeedbackGain = context.constants.translationalCoeffs.kD;
        }
        if (context.constants.angularCoeffs.kD > 0.0) {
            context.constants.angularVelocityFeedbackGain = context.constants.angularCoeffs.kD;
        }
        applyCurrentGains();

        GeometryFactory factory = new GeometryFactory(context.getFollower())
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);

        Pose start = factory.pose(0, 0, 0);
        Pose end = factory.pose(48, 0, 0);
        forwardPath = factory.path(start, end)
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).profiledBuild();
        backwardPath = factory.path(end, start)
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).profiledBuild();

        Pose turned = factory.pose(0, 0, 90);
        forwardTurn = factory.turn(start).turnTo(turned.getHeading()).profiledBuild();
        backwardTurn = factory.turn(turned).turnTo(start.getHeading()).profiledBuild();

        axis = FeedbackAxis.TRANSLATION;
        if (manualMode) {
            startTest();
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
                context.constants.angularVelocityFeedbackGain;
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
            double rawDelta = forwardTurn.getStartPose().getHeading()
                    .getShortestAngleTo(forwardTurn.getEndPose().getHeading()).getRad();
            currentTurnDirection = Math.signum(rawDelta > Math.PI ?
                    rawDelta - 2 * Math.PI : rawDelta);
        }

        context.getFollower().follow(currentMovement);
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
            addError(desired.getTangentialVel(), actual, 1.0);
        } else {
            Turn turn = (Turn) currentMovement;
            double traveled = turn.getStartPose().getHeading().getShortestAngleTo(
                    context.getFollower().getPose().getHeading()
            ).getRad() * currentTurnDirection;

            MotionParameters desired = turn.getFeedforwardLut()
                    .getFFParams(Math.max(0.0, traveled));
            double actual = context.getFollower().getVelocity().getHeading().getRad();
            addError(desired.getAngularVel(), actual, 0.05);
        }
    }

    private void addError(double target, double actual, double minimumTarget) {
        if (Math.abs(target) > minimumTarget) {
            double error = target - actual;
            errorSquared += error * error;
            errorSamples++;
        }
    }

    private boolean updateTest() {
        sampleTest();

        if (context.getFollower().isBusy()) { return false; }

        if (forwardIsRunning) {
            forwardIsRunning = false;

            if (axis == FeedbackAxis.TRANSLATION) {
                currentMovement = backwardPath;
            } else {
                currentMovement = backwardTurn;
            }

            context.getFollower().follow(currentMovement);
            return false;
        }

        lastScore = Math.sqrt(errorSquared / errorSamples);
        return true;
    }

    @Override
    protected boolean autoTuned() {
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
        context.constants.angularVelocityFeedbackGain = center;
        return true;
    }

    @Override
    protected boolean manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed() || opMode.gamepad1.rightBumperWasPressed()) {
            axis = (axis == FeedbackAxis.TRANSLATION) ?
                    FeedbackAxis.ANGULAR : FeedbackAxis.TRANSLATION;
            startTest();
        }

        double change = manualChange();
        if (change != 0.0) {
            if (axis == FeedbackAxis.TRANSLATION) {
                context.constants.velocityFeedbackGain = Math.max(0.0,
                        context.constants.velocityFeedbackGain + change);
            } else {
                context.constants.angularVelocityFeedbackGain = Math.max(0.0,
                        context.constants.angularVelocityFeedbackGain + change);
            }
            applyCurrentGains();
            startTest();
        } else if (opMode.gamepad1.xWasPressed()) {
            startTest();
        } else if (updateTest()) {
            if (axis == FeedbackAxis.TRANSLATION) {
                translationScore = lastScore;
            } else {
                angularScore = lastScore;
            }
            startTest();
        }

        context.getTelemetry().addData("Selected", axis.name());
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Up/Down: change value");
        context.getTelemetry().addLine("Left/Right: change increment");
        context.getTelemetry().addLine("LB/RB: select value");
        context.getTelemetry().addLine("X: restart test");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            return true;
        }

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Translation feedback gain",
                context.constants.velocityFeedbackGain);
        context.getTelemetry().addData("Translation root mean square error", translationScore);
        context.getTelemetry().addData("Angular feedback gain",
                context.constants.angularVelocityFeedbackGain);
        context.getTelemetry().addData("Angular root mean square error", angularScore);
    }
}