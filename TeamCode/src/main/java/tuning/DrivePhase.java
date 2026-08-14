package tuning;

import java.util.function.Supplier;

import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
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
    private double activeTestTarget;
    private boolean testPathQueued;
    private boolean testButtonHeld;
    private final ManualResponseMetrics manualMetrics = new ManualResponseMetrics();

    public DrivePhase(TunerContext context) {
        super(context);

        routine = new PDSRoutine(context, PDSRoutine.Axis.DRIVE);

        GeometryFactory factory = new GeometryFactory(context.getFollower())
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        testPath = () -> factory.path(
                context.getFollower().getPose(),
                context.getFollower().getPose().plus(factory.pose(target, 0.0, 0.0))
        ).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
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
                "Place the robot with at least 36 inches clear in front and behind it.");
        context.getTelemetry().addLine(
                "Automatic tuning runs repeated bounded moves around its start. Response checks " +
                        "move only when requested afterward.");
    }

    @Override
    protected void init() {
        // Both automatic closed-loop trials and the manual test path travel in both directions.
        testPathQueued = false;
        testButtonHeld = opMode.gamepad1.x;
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
        if (manualMetrics.isActive()) {
            manualMetrics.sample(context.getFollower().getPose().getX().getIn(),
                    context.getFollower().getVelocity().getX().getIn(),
                    ManualResponseMetrics.maxMotorPower(
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

        if (opMode.gamepad1.x && !testButtonHeld) { testPathQueued = true; }
        testButtonHeld = opMode.gamepad1.x;
        if (testPathQueued && !context.getFollower().isBusy()) {
            activeTestTarget = target;
            double start = context.getFollower().getPose().getX().getIn();
            manualMetrics.begin(start, start + target, 0.75, 1.0);
            context.getFollower().follow(testPath.get());
            target = -target;
            testPathQueued = false;
        }

        if (opMode.gamepad1.aWasPressed()) {
            manualMetrics.finish();
            return true;
        }

        addTunableValue("Drive P", context.constants.translationalCoeffs.kP,
                selected == Coefficient.P);
        addTunableValue("Drive D", context.constants.translationalCoeffs.kD,
                selected == Coefficient.D);
        addTunableValue("Drive S", context.constants.translationalCoeffs.kS,
                selected == Coefficient.S);
        context.getTelemetry().addData("Increment", number(increment));
        context.getTelemetry().addData("Active Test Target", number(activeTestTarget) + " in");
        context.getTelemetry().addData("Final error", number(manualMetrics.getFinalError()) + " in");
        if (context.isDebugMode()) { reportDetailedManualMetrics(); }
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("LB/RB: Select value to tune");
        context.getTelemetry().addLine("X: Run test path");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Drive P", number(context.constants.translationalCoeffs.kP));
        context.getTelemetry().addData("Drive D", number(context.constants.translationalCoeffs.kD));
        context.getTelemetry().addData("Drive S", number(context.constants.translationalCoeffs.kS));
        if (!manualMode && context.isDebugMode()) {
            context.getTelemetry().addData("Operator check",
                    routine.getOperatorCheckSummary());
        }
    }

    private void reportDetailedManualMetrics() {
        context.getTelemetry().addData("Overshoot", manualMetrics.getOvershoot() + " in");
        context.getTelemetry().addData("Settling time",
                manualMetrics.getSettlingTime() + " s");
        context.getTelemetry().addData("RMS error", manualMetrics.getRmsError() + " in");
        context.getTelemetry().addData("Time-weighted squared error",
                manualMetrics.getTimeWeightedSquaredError());
        context.getTelemetry().addData("Peak velocity",
                manualMetrics.getPeakVelocity() + " in/s");
        context.getTelemetry().addData("Saturation", Math.round(
                manualMetrics.getSaturationFraction() * 1000.0) / 10.0 + "%");
    }
}
