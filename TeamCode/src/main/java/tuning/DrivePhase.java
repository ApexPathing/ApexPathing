package tuning;

import java.util.function.Supplier;

import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Tunes the drive controller used by the follower using a {@link PDSRoutine}. The user can
 * manually tune the coefficients or run the automatic tuning routine.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public class DrivePhase extends TuningPhase {
    private enum Coefficient { P, D, S }

    private final Supplier<Path> testPath;
    private final PDSRoutine routine;

    private Coefficient selected = Coefficient.P;
    private double target = 24.0;

    public DrivePhase(TunerContext context) {
        super(context);

        routine = new PDSRoutine(context, PDSRoutine.Axis.DRIVE);

        GeometryFactory factory = new GeometryFactory(context.getFollower())
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        testPath = () -> factory.path(
                context.getFollower().getPose(),
                context.getFollower().getPose().plus(factory.pose(target, 0.0, 0.0))
        ).interpolateWith(InterpolationStyle.TANGENT_FORWARD).quickBuild();
    }

    @Override
    protected String getPhaseName() { return "Drive Controller"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getTelemetry().addLine(
                "Place the robot with at least 24 inches clear in front and behind it.");
        context.getTelemetry().addLine(
                "The automatic test oscillates around its start, then moves to a nearby target.");
    }

    @Override
    protected void init() {
        positionRobotForSimulation(new Pose(
                Vector.of(-55.0, 0.0, DistUnit.IN), geometry.Angle.fromRad(0.0)));
        // We only want to use the existing drive coefficients if we are in manual mode
        if (manualMode) {
            context.getFollower().enableControllers();
            context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
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

        context.constants.translationalCoeffs = routine.getCoefficients();
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
                context.constants.translationalCoeffs.kP = Math.max(
                        0.0, context.constants.translationalCoeffs.kP + change
                );
            } else if (selected == Coefficient.D) {
                context.constants.translationalCoeffs.kD = Math.max(
                        0.0, context.constants.translationalCoeffs.kD + change
                );
            } else if (selected == Coefficient.S) {
                context.constants.translationalCoeffs.kS = Math.max(
                        0.0, context.constants.translationalCoeffs.kS + change
                );
            }
            context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
        }

        if (opMode.gamepad1.xWasPressed() && !context.getFollower().isBusy()) {
            context.getFollower().follow(testPath.get());
            target = -target;
        }

        if (opMode.gamepad1.aWasPressed()) {
            return true;
        }

        context.getTelemetry().addData("Selected", selected.toString());
        context.getTelemetry().addData("Target", target);
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("Dpad Left/Right: Change increment");
        context.getTelemetry().addLine("LB/RB: Select value to tune");
        context.getTelemetry().addLine(control("X") + ": Run test path");
        context.getTelemetry().addLine(control("A") + ": Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Drive P", context.constants.translationalCoeffs.kP);
        context.getTelemetry().addData("Drive D", context.constants.translationalCoeffs.kD);
        context.getTelemetry().addData("Drive S", context.constants.translationalCoeffs.kS);
        if (!manualMode) {
            context.getTelemetry().addData("Automatic validation",
                    routine.getValidationSummary());
            context.getTelemetry().addData("PDS response CSV", routine.getCsvPath());
        }
    }
}
