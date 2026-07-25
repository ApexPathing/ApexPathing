package tuning;

import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Runs the robot on a forward and backward arc to measure the cross-track error. The error is used
 * to tune the centripetal gain with a binary search. The user can also manually tune the gain.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CentripetalPhase extends TuningPhase {
    private BinarySearch search;
    private Path forwardArc;
    private Path backwardArc;

    private boolean forwardPathRunning;
    private double errorSum;
    private int samples;
    private double averageError;

    public CentripetalPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Centripetal"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);

        Pose start = factory.pose(0, 0, 0);
        Pose end = factory.pose(60, 0, 0);
        forwardArc = factory.path(start, factory.arcPose(30, 30, 40), end)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        backwardArc = factory.path(end, factory.arcPose(30, -30, 40), start)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();

        double fullStrafeAcceleration = context.constants.strafeAccelLimitIn /
                LimitsPhase.MARGIN_MULTIPLIER;
        double seed = context.constants.kCentripetal > 0.0 ?
                context.constants.kCentripetal : 1.0 / fullStrafeAcceleration;
        double upper = Math.max(seed * 2.0, 2.0 / fullStrafeAcceleration);

        search = new BinarySearch(0.0, upper, upper / 64.0);
        context.constants.kCentripetal = manualMode ? seed : search.getGuess();

        resetTrial();
    }

    private void resetTrial() {
        forwardPathRunning = true;
        errorSum = 0;
        samples = 0;
        averageError = 0;

        context.getFollower().setCentripetal(context.constants.kCentripetal);
        context.getFollower().follow(forwardArc);
    }

    private void sampleError() {
        double t = context.getFollower().getBestT();
        if (t <= 0.25 || t >= 0.75) {
            return;
        }

        errorSum += context.getFollower().getCrossTrackErrorIn();
        samples++;
    }

    private boolean updateTrial() {
        if (context.getFollower().isBusy()) {
            sampleError();
            return false;
        }
        if (forwardPathRunning) {
            forwardPathRunning = false;
            context.getFollower().follow(backwardArc);
            return false;
        }

        averageError = errorSum / samples;
        return true;
    }

    @Override
    protected boolean autoTuned() {
        if (!updateTrial()) {
            return false;
        }

        boolean keepSearching = search.updateGuess(averageError > 0.0);
        context.constants.kCentripetal = search.getGuess();
        context.getFollower().setCentripetal(context.constants.kCentripetal);
        if (keepSearching) {
            resetTrial();
        } else {
            return true;
        }

        return false;
    }

    @Override
    protected boolean manualTuned() {
        if (updateTrial()) { // Keep running forever
            resetTrial();
        }

        double change = manualChange();
        if (change != 0.0) {
            context.constants.kCentripetal = Math.max(0.0, context.constants.kCentripetal + change);
            context.getFollower().setCentripetal(context.constants.kCentripetal);
        }

        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Dpad Up/Down: change centripetal gain");
        context.getTelemetry().addLine("Dpad Left/Right: change increment");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        return opMode.gamepad1.aWasPressed();
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Centripetal Gain", context.constants.kCentripetal);
        context.getTelemetry().addData("Mean signed error", averageError);
    }
}
