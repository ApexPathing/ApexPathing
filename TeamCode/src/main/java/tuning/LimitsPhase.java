package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController;
import geometry.Angle;
import geometry.Pose;

/**
 * Measures the maximum velocity and acceleration of the robot in each direction of movement, and
 * derives the follower's velocity and acceleration limits from these measurements.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class LimitsPhase extends TuningPhase {
    enum LimitStage {
        SETTLING,
        RUNNING
    }

    enum LimitTrial {
        FORWARD(1.0, 0.0, 0.0),
        BACKWARD(-1.0, 0.0, 0.0),
        LEFT(0.0, 1.0, 0.0),
        RIGHT(0.0, -1.0, 0.0),
        COUNTERCLOCKWISE(0.0, 0.0, 1.0),
        CLOCKWISE(0.0, 0.0, -1.0);

        final double x, y, turn;

        LimitTrial(double x, double y, double turn) {
            this.x = x;
            this.y = y;
            this.turn = turn;
        }
    }

    private static final LimitTrial[] TRIALS = LimitTrial.values();

    private static final double RUN_TIME = 2000.0;
    private static final double SETTLE_TIME = 800.0;
    public static final double MARGIN_MULTIPLIER = 0.95;

    private final ElapsedTime timer = new ElapsedTime();
    private final double[][] maxima = new double[TRIALS.length][2];

    private PDSController headingHoldController;
    private LimitStage stage = LimitStage.RUNNING;
    private int trial = 0;
    private double heldHeading = 0;

    public LimitsPhase(TunerContext context) {
        super(context);

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
        headingHoldController = new PDSController(context.constants.headingCoeffs);
        headingHoldController.setAngularController();
    }

    private void runTrial() {
        LimitTrial current = TRIALS[trial];
        double turn = current.turn;

        // Apply heading hold correction if we are translating
        if (current != LimitTrial.COUNTERCLOCKWISE && current != LimitTrial.CLOCKWISE) {
            Angle currentHeading = context.getFollower().getPose().getHeading();
            double headingError = currentHeading
                    .getShortestAngleTo(Angle.fromRad(heldHeading)).getRad();
            turn = Math.max(-1.0, Math.min(1.0, headingHoldController.calculate(headingError)));
        }

        context.getFollower().getDrivetrain().moveWithVectors(current.x, current.y, turn);
    }

    private void recordMaximums() {
        Pose velocity = context.getFollower().getVelocity();
        Pose acceleration = context.getFollower().getAcceleration();
        LimitTrial current = TRIALS[trial];
        double measuredVelocity = 0.0;
        double measuredAcceleration = 0.0;

        switch (current) {
            case FORWARD:
            case BACKWARD:
                measuredVelocity = Math.abs(velocity.getX().getIn());
                measuredAcceleration = Math.abs(acceleration.getX().getIn());
                break;
            case LEFT:
            case RIGHT:
                measuredVelocity = Math.abs(velocity.getY().getIn());
                measuredAcceleration = Math.abs(acceleration.getY().getIn());
                break;
            case COUNTERCLOCKWISE:
            case CLOCKWISE:
                measuredVelocity = Math.abs(velocity.getHeading().getRad());
                measuredAcceleration = Math.abs(acceleration.getHeading().getRad());
                break;
        }

        if (Double.isFinite(measuredVelocity)) {
            maxima[trial][0] = Math.max(maxima[trial][0], measuredVelocity);
        }
        if (Double.isFinite(measuredAcceleration)) {
            maxima[trial][1] = Math.max(maxima[trial][1], measuredAcceleration);
        }
    }

    private double weaker(LimitTrial first, LimitTrial second, int measurement) {
        return Math.min(
                maxima[first.ordinal()][measurement], maxima[second.ordinal()][measurement]
        );
    }

    private void deriveValues() {
        double fullForwardVelocity = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 0);
        double fullForwardAcceleration = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 1);
        double fullStrafeVelocity = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 0);
        double fullStrafeAcceleration = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 1);
        double fullAngularVelocity = weaker(LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 0);
        double fullAngularAcceleration = weaker(
                LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 1
        );

        if (fullForwardVelocity <= 0 || fullForwardAcceleration <= 0 ||
                fullStrafeVelocity <= 0 || fullStrafeAcceleration <= 0 ||
                fullAngularVelocity <= 0 || fullAngularAcceleration <= 0) {
            throw new IllegalStateException("One or more of the measured limits is non-positive.");
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
            case SETTLING:
                if (timer.milliseconds() >= SETTLE_TIME) {
                    if (trial >= TRIALS.length) {
                        deriveValues();
                        return true;
                    }
                    heldHeading = context.getFollower().getPose().getHeading().getRad();
                    headingHoldController.reset();
                    timer.reset();
                    stage = LimitStage.RUNNING;
                }
                break;
            case RUNNING:
                runTrial();
                recordMaximums();
                if (timer.milliseconds() >= RUN_TIME) {
                    context.getFollower().stop();
                    trial++;
                    timer.reset();
                    stage = LimitStage.SETTLING;
                }
                break;
        }

        String step = trial >= TRIALS.length ? "Calculating" : TRIALS[trial].name();
        context.getTelemetry().addData("Step", step);
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