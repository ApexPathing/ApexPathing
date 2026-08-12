package tuning;

import geometry.Angle;
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
    private double nextTestTarget = 90.0;
    private boolean testTurnQueued = false;

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
                "Place the robot where it can rotate safely about 80 degrees in either direction.");
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
                    .turnTo(Angle.fromDeg(activeTestTarget))
                    .quickBuild();
            context.getFollower().follow(testTurn);
            nextTestTarget = -nextTestTarget;
            testTurnQueued = false;
        }

        if (opMode.gamepad1.aWasPressed()) {
            return true;
        }

        context.getTelemetry().addData("Selected", selected.toString());
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addData("Active Test Target", activeTestTarget + " deg");
        context.getTelemetry().addData("Next Test Target", nextTestTarget + " deg");
        context.getTelemetry().addData("Test Turn Queued", testTurnQueued);
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("Dpad Left/Right: Change increment");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine(control("X") + ": Run test turn");
        context.getTelemetry().addLine(control("A") + ": Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", context.constants.angularCoeffs.kP);
        context.getTelemetry().addData("Heading D", context.constants.angularCoeffs.kD);
        context.getTelemetry().addData("Heading S", context.constants.angularCoeffs.kS);
        if (!manualMode) {
            context.getTelemetry().addData("Automatic validation",
                    routine.getValidationSummary());
            context.getTelemetry().addData("PDS response CSV", routine.getCsvPath());
        }
    }
}
