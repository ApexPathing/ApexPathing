package tuning;

import geometry.Angle;
import geometry.AngleUnit;
import paths.builders.TurnBuilder;
import paths.movements.Turn;

/**
 * Tunes the heading controller used by the follower using a {@link PDSRoutine}. The user can
 * manually tune the coefficients or run the automatic tuning routine.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class HeadingPhase extends TuningPhase {
    private enum Coefficient { P, D, S }

    private final PDSRoutine routine;

    private Coefficient selected = Coefficient.P;
    private double activeTestTarget = 0.0;
    private double nextTestTarget = 60.0;
    private boolean testTurnQueued = false;
    private Angle manualHeadingOrigin = Angle.fromRad(0.0);
    private final ManualResponseMetrics manualMetrics = new ManualResponseMetrics();

    public HeadingPhase(TunerContext context) {
        super(context);

        routine = new PDSRoutine(context, PDSRoutine.Axis.HEADING);
    }

    @Override
    protected String getPhaseName() { return "Heading Controller"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Place the robot where it can rotate safely through a 60 degree out-and-back test.");
    }

    @Override
    protected void init() {
        positionRobotForSimulation(geometry.Pose.zero());
        // Clear any motor command left by the previously selected phase before changing which
        // controllers are allowed to write to the drivetrain.
        context.getFollower().stop();

        // We only want to use the existing heading coefficients if we are in manual mode
        if (manualMode) {
            // Heading tuning must be a pure point turn. Do not let position hold turn
            // localization drift into x/y drivetrain power.
            context.getFollower().enableHeadingController();
            context.getFollower().disableDriveController();
            context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
            manualHeadingOrigin = context.getFollower().getPose().getHeading();
            activeTestTarget = 0.0;
            nextTestTarget = 60.0;
            testTurnQueued = false;
            return;
        }

        routine.start(context);
    }

    @Override
    protected boolean autoTuned() {
        if (!routine.update(context)) {
            routine.reportProgress(context);
            return false;
        }

        context.constants.angularCoeffs = routine.getCoefficients();
        return true;
    }

    @Override
    protected boolean manualTuned() {
        if (manualMetrics.isActive()) {
            double position = manualHeadingOrigin.getShortestAngleTo(
                    context.getFollower().getPose().getHeading()).getRad();
            double velocity = context.getFollower().getVelocity().getHeading(AngleUnit.RAD);
            manualMetrics.sample(position, velocity, ManualResponseMetrics.maxMotorPower(
                    context.getFollower().getDrivetrain()));
            if (!context.getFollower().isBusy()) { manualMetrics.finish(); }
        }
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = selected == Coefficient.P ? Coefficient.S :
                    Coefficient.values()[selected.ordinal() - 1];
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = selected == Coefficient.S ? Coefficient.P :
                    Coefficient.values()[selected.ordinal() + 1];
        }

        double change = manualChange();
        if (change != 0.0) {
            if (selected == Coefficient.P) {
                context.constants.angularCoeffs.kP = Math.max(
                        0.0, context.constants.angularCoeffs.kP + change
                );
            } else if (selected == Coefficient.D) {
                context.constants.angularCoeffs.kD = Math.max(
                        0.0, context.constants.angularCoeffs.kD + change
                );
            } else if (selected == Coefficient.S) {
                context.constants.angularCoeffs.kS = Math.max(
                        0.0, context.constants.angularCoeffs.kS + change
                );
            }
            context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
        }

        if (opMode.gamepad1.xWasPressed()) {
            testTurnQueued = true;
        }
        if (testTurnQueued && !context.getFollower().isBusy()) {
            activeTestTarget = nextTestTarget;
            Turn testTurn = new TurnBuilder(context.getFollower().getPose())
                    .turnTo(Angle.fromRad(manualHeadingOrigin.getRad() +
                            Math.toRadians(activeTestTarget)))
                    .quickBuild();
            double start = manualHeadingOrigin.getShortestAngleTo(
                    context.getFollower().getPose().getHeading()).getRad();
            manualMetrics.begin("manual_heading_response", start,
                    Math.toRadians(activeTestTarget), Math.toRadians(2.5), 0.10);
            context.getFollower().follow(testTurn);
            nextTestTarget = nextManualTestTarget(activeTestTarget);
            testTurnQueued = false;
        }

        if (opMode.gamepad1.aWasPressed()) {
            manualMetrics.finish();
            return true;
        }

        addTunableValue("Heading P", context.constants.angularCoeffs.kP,
                selected == Coefficient.P);
        addTunableValue("Heading D", context.constants.angularCoeffs.kD,
                selected == Coefficient.D);
        addTunableValue("Heading S", context.constants.angularCoeffs.kS,
                selected == Coefficient.S);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Active Test Target", number(activeTestTarget) + " deg");
        context.getTelemetry().addData("Final error", number(manualMetrics.getFinalError()) + " rad");
        if (context.isDebugMode()) { reportDetailedManualMetrics("rad", "rad/s"); }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("X: Run test turn");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", number(context.constants.angularCoeffs.kP));
        context.getTelemetry().addData("Heading D", number(context.constants.angularCoeffs.kD));
        context.getTelemetry().addData("Heading S", number(context.constants.angularCoeffs.kS));
        if (!manualMode && context.isDebugMode()) {
            context.getTelemetry().addData("Operator check",
                    routine.getOperatorCheckSummary());
            context.getTelemetry().addData("PDS response CSV", routine.getCsvPath());
        }
    }

    private void reportDetailedManualMetrics(String positionUnit, String velocityUnit) {
        context.getTelemetry().addData("Overshoot",
                manualMetrics.getOvershoot() + " " + positionUnit);
        context.getTelemetry().addData("Settling time",
                manualMetrics.getSettlingTime() + " s");
        context.getTelemetry().addData("RMS error",
                manualMetrics.getRmsError() + " " + positionUnit);
        context.getTelemetry().addData("Time-weighted squared error",
                manualMetrics.getTimeWeightedSquaredError());
        context.getTelemetry().addData("Peak velocity",
                manualMetrics.getPeakVelocity() + " " + velocityUnit);
        context.getTelemetry().addData("Saturation", Math.round(
                manualMetrics.getSaturationFraction() * 1000.0) / 10.0 + "%");
        context.getTelemetry().addData("Response CSV", manualMetrics.getCsvPath());
    }

    static double nextManualTestTarget(double completedTargetDegrees) {
        return Math.abs(completedTargetDegrees) < 1e-9 ? 60.0 : 0.0;
    }
}
