package tuning;

import java.util.function.Supplier;

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

    private final Supplier<Turn> testTurn;
    private final PDSRoutine routine;

    private Coefficient selected = Coefficient.P;
    private double target = 90.0;

    public HeadingPhase(TunerContext context) {
        super(context);

        routine = new PDSRoutine(context, PDSRoutine.Axis.HEADING);
        testTurn = () -> new TurnBuilder(context.getFollower().getPose())
                .turnTo(Angle.fromDeg(target))
                .quickBuild();
    }

    @Override
    protected String getPhaseName() { return "Heading Controller"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        // We only want to use the existing heading coefficients if we are in manual mode
        if (manualMode) {
            context.getFollower().setHeadingCoefficients(context.constants.headingCoeffs);
            return;
        }

        routine.start();
    }

    @Override
    protected boolean autoTuned() {
        if (!routine.update(context)) {
            return false;
        }

        context.constants.headingCoeffs = routine.getCoefficients();
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
                context.constants.headingCoeffs.kP = Math.max(
                        0.0, context.constants.headingCoeffs.kP + change
                );
            } else if (selected == Coefficient.D) {
                context.constants.headingCoeffs.kD = Math.max(
                        0.0, context.constants.headingCoeffs.kD + change
                );
            } else if (selected == Coefficient.S) {
                context.constants.headingCoeffs.kS = Math.max(
                        0.0, context.constants.headingCoeffs.kS + change
                );
            }
        }

        if (opMode.gamepad1.xWasPressed() && !context.getFollower().isBusy()) {
            context.getFollower().follow(testTurn.get());
            target = -target;
        }

        if (opMode.gamepad1.aWasPressed()) {
            return true;
        }

        context.getTelemetry().addData("Selected", selected.toString());
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Dpad Up/Down: change value");
        context.getTelemetry().addLine("Dpad Left/Right: change increment");
        context.getTelemetry().addLine("LB/RB: select value to tune");
        context.getTelemetry().addLine("X: test turn");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", context.constants.headingCoeffs.kP);
        context.getTelemetry().addData("Heading D", context.constants.headingCoeffs.kD);
        context.getTelemetry().addData("Heading S", context.constants.headingCoeffs.kS);
    }
}
