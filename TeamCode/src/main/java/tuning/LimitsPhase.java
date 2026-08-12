package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

import controllers.PDSController;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

/**
 * Measures the maximum velocity and acceleration of the robot on each movement axis, and
 * derives the follower's velocity and acceleration limits from these measurements.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class LimitsPhase extends TuningPhase {
    enum LimitStage {
        PROMPT,
        SETTLING,
        RUNNING
    }

    enum LimitTrial {
        FORWARD(1.0, 0.0, 0.0),
        LEFT(0.0, 1.0, 0.0),
        COUNTERCLOCKWISE(0.0, 0.0, 1.0);

        final double x, y, turn;

        LimitTrial(double x, double y, double turn) {
            this.x = x;
            this.y = y;
            this.turn = turn;
        }
    }

    private static final LimitTrial[] TRIALS = LimitTrial.values();

    private static final double MIN_RUN_TIME = 500.0;
    private static final double MAX_RUN_TIME = 2500.0;
    private static final double SETTLE_TIME = 1000.0;
    private static final double ACCELERATION_IGNORE_TIME = 60.0;
    private static final int VELOCITY_WINDOW_SIZE = 15;
    private static final int REQUIRED_STABLE_WINDOWS = 3;
    private static final double MAX_TRANSLATION_TRAVEL = 60.0;
    private static final double MAX_ANGULAR_TRAVEL = Math.PI * 2.0;
    private static final double SIM_STAGING_OFFSET = 55.0;
    public static final double MARGIN_MULTIPLIER = 0.95;

    private final ElapsedTime timer = new ElapsedTime();
    private final double[][] maxima = new double[TRIALS.length][2];

    private PDSController headingHoldController;
    private final Deque<TimedSample> velocityWindow = new ArrayDeque<>();
    private final List<Double> velocitySamples = new ArrayList<>();
    private final List<Double> accelerationSamples = new ArrayList<>();

    private LimitStage stage = LimitStage.PROMPT;
    private int trial = 0;
    private double heldHeading = 0;
    private double travelled = 0.0;
    private double lastSampleTime = 0.0;
    private double lastMeasuredVelocity = Double.NaN;
    private int stableWindows = 0;

    static final class TimedSample {
        final double timeSeconds;
        final double value;

        TimedSample(double timeSeconds, double value) {
            this.timeSeconds = timeSeconds;
            this.value = value;
        }
    }

    public LimitsPhase(TunerContext context) {
        super(context);
        this.context.getFollower().getDrivetrain().getConstants().maxPower = 1.0;

        for (double[] maximum : maxima) {
            maximum[0] = 0.0;
            maximum[1] = 0.0;
        }
    }

    @Override
    protected String getPhaseName() { return "Movement Limits"; }

    @Override
    protected boolean manualTuneIsPossible() { return false; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        headingHoldController = new PDSController(context.constants.angularCoeffs);
        headingHoldController.setAngularController();
        stage = LimitStage.PROMPT;
        trial = 0;
        timer.reset();
    }

    private boolean isRotation(LimitTrial value) {
        return value == LimitTrial.COUNTERCLOCKWISE;
    }

    private String directionDescription(LimitTrial value) {
        switch (value) {
            case FORWARD: return "the front of the robot toward clear space";
            case LEFT: return "the left side of the robot toward clear space";
            default: return "the robot where it can rotate without striking anything";
        }
    }

    private void beginTrial() {
        LimitTrial current = TRIALS[trial];
        if (current == LimitTrial.FORWARD) {
            positionRobotForSimulation(stagingPose(-SIM_STAGING_OFFSET, 0.0));
        } else if (current == LimitTrial.LEFT) {
            positionRobotForSimulation(stagingPose(0.0, -SIM_STAGING_OFFSET));
        } else {
            positionRobotForSimulation(Pose.zero());
        }
        heldHeading = context.getFollower().getPose().getHeading().getRad();
        headingHoldController.reset();
        velocityWindow.clear();
        velocitySamples.clear();
        accelerationSamples.clear();
        travelled = 0.0;
        lastSampleTime = 0.0;
        lastMeasuredVelocity = Double.NaN;
        stableWindows = 0;
        timer.reset();
        stage = LimitStage.SETTLING;
    }

    private Pose stagingPose(double x, double y) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(0.0));
    }

    private void runTrial() {
        LimitTrial current = TRIALS[trial];
        double turn = current.turn;

        // Apply heading hold correction if we are translating
        if (!isRotation(current)) {
            Angle currentHeading = context.getFollower().getPose().getHeading();
            double headingError = currentHeading
                    .getShortestAngleTo(Angle.fromRad(heldHeading)).getRad();
            turn = Math.max(-1.0, Math.min(1.0, headingHoldController.calculate(headingError)));
        }

        context.getFollower().getDrivetrain().moveWithVectors(current.x, current.y, turn);
    }

    private double readVelocity() {
        Pose velocity = context.getFollower().getVelocity();
        LimitTrial current = TRIALS[trial];
        double measuredVelocity = 0.0;

        switch (current) {
            case FORWARD:
                measuredVelocity = Math.abs(velocity.getX().getIn());
                break;
            case LEFT:
                measuredVelocity = Math.abs(velocity.getY().getIn());
                break;
            case COUNTERCLOCKWISE:
                measuredVelocity = Math.abs(velocity.getHeading().getRad());
                break;
        }

        return measuredVelocity;
    }

    private boolean recordAndCheckTrial() {
        double elapsedSeconds = timer.milliseconds() / 1000.0;
        double velocity = readVelocity();

        if (Double.isFinite(velocity)) {
            velocitySamples.add(velocity);
            if (lastSampleTime > 0.0) {
                double deltaTime = elapsedSeconds - lastSampleTime;
                travelled += velocity * Math.max(0.0, deltaTime);

                // Derive acceleration from the same timestamped velocity stream used to detect
                // maximum velocity. The localizer's separately filtered acceleration can miss a
                // short FTCodeSim/robot ramp and report an entire valid trial as zero.
                if (timer.milliseconds() >= ACCELERATION_IGNORE_TIME && deltaTime > 0.005 &&
                        Double.isFinite(lastMeasuredVelocity)) {
                    double derivedAcceleration = (velocity - lastMeasuredVelocity) / deltaTime;
                    if (Double.isFinite(derivedAcceleration) && derivedAcceleration > 0.0) {
                        accelerationSamples.add(derivedAcceleration);
                    }
                }
            }
            velocityWindow.addLast(new TimedSample(elapsedSeconds, velocity));
            while (velocityWindow.size() > VELOCITY_WINDOW_SIZE) {
                velocityWindow.removeFirst();
            }
            lastSampleTime = elapsedSeconds;
            lastMeasuredVelocity = velocity;
        }

        double[] allVelocities = toArray(velocitySamples);
        double robustHighVelocity = percentile(allVelocities, 0.95);
        double currentMedian = windowMedian(velocityWindow);
        double movementFloor = isRotation(TRIALS[trial]) ? 0.10 : 1.0;
        boolean plateau = timer.milliseconds() >= MIN_RUN_TIME &&
                currentMedian >= Math.max(movementFloor, robustHighVelocity * 0.50) &&
                isVelocityPlateau(velocityWindow, isRotation(TRIALS[trial]));
        stableWindows = plateau ? stableWindows + 1 : 0;
        double travelLimit = isRotation(TRIALS[trial])
                ? MAX_ANGULAR_TRAVEL : MAX_TRANSLATION_TRAVEL;
        return stableWindows >= REQUIRED_STABLE_WINDOWS ||
                timer.milliseconds() >= MAX_RUN_TIME || travelled >= travelLimit;
    }

    private void finishTrial() {
        context.getFollower().stop();
        // Use the complete run rather than its final window. A collision or late stop must not
        // replace a valid maximum-speed measurement with zero.
        maxima[trial][0] = percentile(toArray(velocitySamples), 0.95);
        double[] accelerations = toArray(accelerationSamples);
        // Acceleration is a differentiated signal; the 95th percentile captures the strong part
        // of the ramp without allowing one noisy sample to define the robot's limit.
        maxima[trial][1] = percentile(accelerations, 0.95);

        trial++;
        timer.reset();
        stage = LimitStage.PROMPT;
    }

    private static double[] toArray(List<Double> values) {
        double[] result = new double[values.size()];
        for (int i = 0; i < result.length; i++) { result[i] = values.get(i); }
        return result;
    }

    private static double windowMedian(Deque<TimedSample> samples) {
        double[] values = new double[samples.size()];
        int index = 0;
        for (TimedSample sample : samples) { values[index++] = sample.value; }
        return percentile(values, 0.50);
    }

    static double percentile(double[] values, double percentile) {
        if (values.length == 0) { return 0.0; }
        double[] sorted = Arrays.copyOf(values, values.length);
        Arrays.sort(sorted);
        double position = Math.max(0.0, Math.min(1.0, percentile)) * (sorted.length - 1);
        int lower = (int) Math.floor(position);
        int upper = (int) Math.ceil(position);
        if (lower == upper) { return sorted[lower]; }
        double fraction = position - lower;
        return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
    }

    static boolean isVelocityPlateau(Deque<TimedSample> samples, boolean angular) {
        if (samples.size() < VELOCITY_WINDOW_SIZE) { return false; }
        double meanTime = 0.0;
        double meanVelocity = 0.0;
        double[] velocities = new double[samples.size()];
        int i = 0;
        for (TimedSample sample : samples) {
            meanTime += sample.timeSeconds;
            meanVelocity += sample.value;
            velocities[i++] = sample.value;
        }
        meanTime /= samples.size();
        meanVelocity /= samples.size();

        double covariance = 0.0;
        double timeVariance = 0.0;
        for (TimedSample sample : samples) {
            covariance += (sample.timeSeconds - meanTime) * (sample.value - meanVelocity);
            timeVariance += (sample.timeSeconds - meanTime) * (sample.timeSeconds - meanTime);
        }
        double slope = timeVariance > 1e-9 ? covariance / timeVariance : Double.POSITIVE_INFINITY;
        double median = percentile(velocities, 0.5);
        for (i = 0; i < velocities.length; i++) {
            velocities[i] = Math.abs(velocities[i] - median);
        }
        double mad = percentile(velocities, 0.5);
        double absoluteSlopeLimit = angular ? 0.10 : 0.50;
        double slopeLimit = Math.max(absoluteSlopeLimit, Math.abs(median) * 0.03);
        double spreadLimit = Math.max(angular ? 0.02 : 0.20, Math.abs(median) * 0.05);
        return Math.abs(slope) <= slopeLimit && mad <= spreadLimit;
    }

    private void deriveValues() {
        double fullForwardVelocity = maxima[LimitTrial.FORWARD.ordinal()][0];
        double fullForwardAcceleration = maxima[LimitTrial.FORWARD.ordinal()][1];
        double fullStrafeVelocity = maxima[LimitTrial.LEFT.ordinal()][0];
        double fullStrafeAcceleration = maxima[LimitTrial.LEFT.ordinal()][1];
        double fullAngularVelocity = maxima[LimitTrial.COUNTERCLOCKWISE.ordinal()][0];
        double fullAngularAcceleration = maxima[LimitTrial.COUNTERCLOCKWISE.ordinal()][1];

        if (fullForwardVelocity <= 0 || fullForwardAcceleration <= 0 ||
                fullStrafeVelocity <= 0 || fullStrafeAcceleration <= 0 ||
                fullAngularVelocity <= 0 || fullAngularAcceleration <= 0) {
            throw new IllegalStateException("One or more measured limits is non-positive. " +
                    "Forward=" + Arrays.toString(maxima[LimitTrial.FORWARD.ordinal()]) +
                    ", Left=" + Arrays.toString(maxima[LimitTrial.LEFT.ordinal()]) +
                    ", CCW=" + Arrays.toString(
                            maxima[LimitTrial.COUNTERCLOCKWISE.ordinal()]));
        }

        context.constants.forwardVelLimitIn = fullForwardVelocity * MARGIN_MULTIPLIER;
        context.constants.forwardAccelLimitIn = fullForwardAcceleration * MARGIN_MULTIPLIER;
        context.constants.strafeVelLimitIn = fullStrafeVelocity * MARGIN_MULTIPLIER;
        context.constants.strafeAccelLimitIn = fullStrafeAcceleration * MARGIN_MULTIPLIER;
        context.constants.angularVelLimitRad = fullAngularVelocity * MARGIN_MULTIPLIER;
        context.constants.angularAccelLimitRad = fullAngularAcceleration * MARGIN_MULTIPLIER;

        context.constants.translationalKV = 1.0 / fullForwardVelocity;
        context.constants.translationalKA = 1.0 / fullForwardAcceleration;
        context.constants.angularKV = 1.0 / fullAngularVelocity;
        context.constants.angularKA = 1.0 / fullAngularAcceleration;
    }

    @Override
    protected boolean autoTuned() {
        switch (stage) {
            case PROMPT:
                context.getFollower().stop();
                if (trial >= TRIALS.length) {
                    deriveValues();
                    return true;
                }
                LimitTrial promptedTrial = TRIALS[trial];
                context.getTelemetry().addLine("Next limits run: " + promptedTrial.name());
                context.getTelemetry().addLine("Point " + directionDescription(promptedTrial) + ".");
                context.getTelemetry().addLine(isRotation(promptedTrial)
                        ? "Allow a full clear rotation around the robot."
                        : "Allow at least 72 inches of clear travel (the powered run stops by 60)."
                );
                context.getTelemetry().addLine("Press " + control("A") +
                        " when the robot is stationary and the direction is safe.");
                context.getTelemetry().update();
                if (opMode.gamepad1.aWasPressed()) { beginTrial(); }
                return false;
            case SETTLING:
                if (timer.milliseconds() >= SETTLE_TIME) {
                    timer.reset();
                    stage = LimitStage.RUNNING;
                }
                break;
            case RUNNING:
                runTrial();
                if (recordAndCheckTrial()) { finishTrial(); }
                break;
        }

        String step = trial >= TRIALS.length ? "Calculating" : TRIALS[trial].name();
        context.getTelemetry().addData("Step", step);
        if (stage == LimitStage.RUNNING) {
            context.getTelemetry().addData("Stop detection",
                    stableWindows + " / " + REQUIRED_STABLE_WINDOWS + " stable windows");
            context.getTelemetry().addData("Travel", travelled);
        }
        context.getTelemetry().update();
        return false;
    }

    @Override
    protected boolean manualTuned() { return true; }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Forward Velocity", context.constants.forwardVelLimitIn);
        context.getTelemetry()
                .addData("Forward Acceleration", context.constants.forwardAccelLimitIn);
        context.getTelemetry().addData("Strafe Velocity", context.constants.strafeVelLimitIn);
        context.getTelemetry().addData("Strafe Acceleration", context.constants.strafeAccelLimitIn);
        context.getTelemetry().addData("Angular Velocity", context.constants.angularVelLimitRad);
        context.getTelemetry()
                .addData("Angular Acceleration", context.constants.angularAccelLimitRad);
        context.getTelemetry().addData("Translation kV", context.constants.translationalKV);
        context.getTelemetry().addData("Translation kA", context.constants.translationalKA);
        context.getTelemetry().addData("Angular kV", context.constants.angularKV);
        context.getTelemetry().addData("Angular kA", context.constants.angularKA);
    }
}
