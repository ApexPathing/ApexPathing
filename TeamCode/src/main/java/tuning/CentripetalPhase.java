package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    private static final double LEG_TIMEOUT_SECONDS = 15.0;
    private static final double TURNAROUND_PROGRESS = 0.985;
    private static final double TURNAROUND_DISTANCE_INCHES = 1.5;
    private static final double TURNAROUND_SPEED_INCHES_PER_SECOND = 6.0;

    private BinarySearch search;
    private Path forwardArc;
    private Path backwardArc;

    private boolean forwardPathRunning;
    private double errorSum;
    private int samples;
    private double averageError;
    private boolean trialRunning;
    private int trialNumber;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private final ElapsedTime legTimer = new ElapsedTime();

    public CentripetalPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Centripetal"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Place the robot at one corner of a clear 32 by 32 inch test area.");
        context.getTelemetry().addLine(
                "Manual arc tests remain stopped until X is pressed.");
    }

    @Override
    protected void init() {
        GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);

        // Center the complete 32x32 test footprint on the field, not merely its starting point.
        Pose start = factory.pose(-16, -16, 0);
        Pose middle = factory.pose(16, -16, 0);
        Pose end = factory.pose(16, 16, 90);
        context.getFollower().setPose(start);
        forwardArc = factory.path(start, middle, end)
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).quickBuild();
        backwardArc = factory.path(end, middle, start)
                .interpolateWith(InterpolationStyle.TANGENT_BACKWARD).quickBuild();

        double fullStrafeAcceleration = context.constants.strafeAccelLimitIn /
                LimitsPhase.MARGIN_MULTIPLIER;
        double seed = context.constants.kCentripetal > 0.0 ?
                context.constants.kCentripetal : 1.0 / fullStrafeAcceleration;
        double upper = Math.max(seed * 2.0, 2.0 / fullStrafeAcceleration);

        search = new BinarySearch(0.0, upper, upper / 64.0);
        context.constants.kCentripetal = manualMode ? seed : search.getGuess();

        if (manualMode) {
            context.getFollower().setCentripetal(context.constants.kCentripetal);
            context.getFollower().stop();
            trialRunning = false;
            trialNumber = 0;
            manualCsv = TuningCsvWriter.open("manual_centripetal_response",
                    "trial", "time_s", "gain", "direction", "path_t", "signed_error_in");
            manualCsvPath = manualCsv.getPath();
        } else {
            resetTrial();
        }
    }

    private void resetTrial() {
        context.getFollower().stop();
        forwardPathRunning = true;
        errorSum = 0;
        samples = 0;
        averageError = 0;

        context.getFollower().setCentripetal(context.constants.kCentripetal);
        context.getFollower().follow(forwardArc);
        trialRunning = true;
        trialNumber++;
        legTimer.reset();
    }

    private void sampleError() {
        double t = context.getFollower().getBestT();
        if (t <= 0.25 || t >= 0.75) {
            return;
        }

        double error = context.getFollower().getCentripetalErrorIn();
        if (!Double.isFinite(error)) {
            return;
        }
        errorSum += error;
        samples++;
        if (manualMode && manualCsv != null) {
            manualCsv.writeRow(trialNumber, legTimer.seconds(),
                    context.constants.kCentripetal,
                    forwardPathRunning ? "OUTBOUND" : "RETURN", t, error);
        }
    }

    private boolean updateTrial() {
        if (context.getFollower().isBusy()) {
            sampleError();
            if (readyForTurnaround()) {
                return finishLeg();
            }
            if (legTimer.seconds() > LEG_TIMEOUT_SECONDS) {
                context.getFollower().stop();
                throw new IllegalStateException(
                        "Centripetal " + (forwardPathRunning ? "outbound" : "return") +
                                " arc timed out at pose " + context.getFollower().getPose() +
                                ", t=" + context.getFollower().getBestT() +
                                ", velocity=" + context.getFollower().getVelocity() +
                                ". Verify endpoint tolerances, localization, and PDS constants."
                );
            }
            return false;
        }
        return finishLeg();
    }

    private boolean readyForTurnaround() {
        Pose endpoint = forwardPathRunning ? forwardArc.getEndPose() : backwardArc.getEndPose();
        double endpointDistance = context.getFollower().getPose().getVec()
                .distanceTo(endpoint.getVec()).getIn();
        double speed = context.getFollower().getVelocity().getVec().getMag().getIn();
        return readyForTurnaround(context.getFollower().getBestT(), endpointDistance, speed);
    }

    static boolean readyForTurnaround(double progress, double endpointDistance, double speed) {
        return Double.isFinite(progress) && Double.isFinite(endpointDistance) &&
                Double.isFinite(speed) && progress >= TURNAROUND_PROGRESS &&
                endpointDistance <= TURNAROUND_DISTANCE_INCHES &&
                speed <= TURNAROUND_SPEED_INCHES_PER_SECOND;
    }

    private boolean finishLeg() {
        // The general follower completion gate may still be waiting on heading settling. End this
        // phase-local leg explicitly so the matching reverse arc can be queued immediately.
        if (context.getFollower().isBusy()) { context.getFollower().stop(); }
        if (forwardPathRunning) {
            forwardPathRunning = false;
            context.getFollower().follow(backwardArc);
            legTimer.reset();
            return false;
        }

        if (samples == 0) {
            throw new IllegalStateException(
                    "Centripetal trial completed without usable middle-arc samples. " +
                            "Verify localization and path tracking before tuning kCentripetal."
            );
        }
        averageError = errorSum / samples;
        return true;
    }

    @Override
    protected boolean autoTuned() {
        context.getTelemetry().addLine(forwardPathRunning
                ? "Robot is following the outbound test arc."
                : "Robot is following the return test arc.");
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Current Pose", context.getFollower().getPose().toString());
            context.getTelemetry().addData("Follower T", context.getFollower().getBestT());
            context.getTelemetry().addData("Follower Cross Track Error", context.getFollower().getCrossTrackErrorIn());
            context.getTelemetry().addData("Inward Centripetal Error",
                    context.getFollower().getCentripetalErrorIn());
            context.getTelemetry().addData("Closest Path Point",
                    context.getFollower().getClosestPathPoint().toString());
            context.getTelemetry().addData("Cross-Track Normal",
                    context.getFollower().getCrossTrackNormal().toString());
            context.getTelemetry().addData("Inward Curve Normal",
                    context.getFollower().getPathNormal().toString());
            context.getTelemetry().addData("Cross-Track Vector",
                    context.getFollower().getCrossTrackCorrection().toString());
            context.getTelemetry().addData("Centripetal Vector",
                    context.getFollower().getCentripetalCorrection().toString());
            context.getTelemetry().addData("Average Error", averageError);
            context.getTelemetry().addData("Direction",
                    forwardPathRunning ? "OUTBOUND" : "RETURN");
            context.getTelemetry().addData("Leg elapsed",
                    Math.round(legTimer.seconds() * 10.0) / 10.0 + " / " +
                            LEG_TIMEOUT_SECONDS + " s");
            context.getTelemetry().addData("Current Velocity",
                    context.getFollower().getVelocity().toString());
        }
        context.getTelemetry().update();

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
        double change = manualChange();
        if (change != 0.0) {
            context.constants.kCentripetal = Math.max(0.0, context.constants.kCentripetal + change);
            context.getFollower().setCentripetal(context.constants.kCentripetal);
            if (trialRunning) { resetTrial(); }
        }

        if (opMode.gamepad1.xWasPressed()) {
            resetTrial();
        } else if (trialRunning && updateTrial()) {
            trialRunning = false;
            context.getFollower().stop();
        }

        reportResults();
        context.getTelemetry().addData("Increment", number(increment));
        if (context.isDebugMode()) {
            context.getTelemetry().addData("Test state", trialRunning
                    ? (forwardPathRunning ? "OUTBOUND" : "RETURN")
                    : "IDLE - press X to run");
            context.getTelemetry().addData("Usable samples", samples);
            context.getTelemetry().addData("Response CSV", manualCsvPath);
        }
        context.getTelemetry().addLine("Dpad Up/Down: change centripetal gain");
        context.getTelemetry().addLine("X: run/restart arc test");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            if (manualCsv != null) { manualCsv.close(); }
            return true;
        }
        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Centripetal Gain", number(context.constants.kCentripetal));
        context.getTelemetry().addData("Mean signed error",
                number(trialRunning && samples > 0 ? errorSum / samples : averageError));
    }
}
