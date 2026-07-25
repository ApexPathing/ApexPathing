package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Base class for tuning phases for the follower tuner. Each phase is responsible for tuning a
 * specific aspect of the follower's behavior The class provides a framework for running the tuning
 * process, including selecting the tuning mode (manual or automatic), executing the tuning logic,
 * and displaying results.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public abstract class TuningPhase {
    enum TuningState {
        SELECT_MODE,
        TUNING,
        RESULTS
    }

    protected final TunerContext context;
    protected LinearOpMode opMode;
    protected boolean manualMode;
    protected double increment;

    protected TuningPhase(TunerContext context) { this.context = context; }

    public boolean run(LinearOpMode opMode) {
        this.opMode = opMode;
        manualMode = false;
        increment = 0.001;

        TuningState state = TuningState.SELECT_MODE;

        while (opMode.opModeIsActive()) {
            switch (state) {
                case SELECT_MODE:
                    showModeSelector();
                    if (manualTuneIsPossible() && autoTuneIsPossible() &&
                            opMode.gamepad1.bWasPressed()) {
                        manualMode = !manualMode;
                    }
                    if (opMode.gamepad1.aWasPressed()) {
                        init();
                        state = TuningState.TUNING;
                    }
                    break;
                case TUNING:
                    context.getFollower().update();
                    boolean complete;
                    if (manualMode) {
                        complete = manualTuned();
                    } else {
                        complete = autoTuned();
                    }
                    if (complete) {
                        context.getFollower().stop();
                        state = TuningState.RESULTS;
                    }
                    break;
                case RESULTS:
                    showResults();
                    if (opMode.gamepad1.bWasPressed()) {
                        return true;
                    }
                    break;
            }
        }

        context.getFollower().stop();
        return false;
    }

    private void showModeSelector() {
        context.getTelemetry().addLine(getPhaseName() + " phase initialized");
        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getTelemetry().addLine("Press B to toggle automatic and manual tuning.");
            context.getTelemetry().addData("Selected Mode:", manualMode ? "Manual" : "Automatic");
        } else {
            manualMode = manualTuneIsPossible();
            context.getTelemetry().addData("Tuner Type:", manualMode ? "Manual" : "Automatic");
        }
        context.getTelemetry().addLine("Press A to run this phase.");
        context.getTelemetry().update();
    }

    private void showResults() {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with results:");
        reportResults();
        context.getTelemetry().addLine("Press B to continue.");
        context.getTelemetry().update();
    }

    protected double manualChange() {
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            increment = Math.max(increment / 10.0, 0.00001);
        } else if (opMode.gamepad1.dpadRightWasPressed()) {
            increment = Math.min(increment * 10.0, 1.0);
        } else if (opMode.gamepad1.dpadUpWasPressed()) {
            return increment;
        } else if (opMode.gamepad1.dpadDownWasPressed()) {
            return -increment;
        }
        return 0.0;
    }

    protected abstract String getPhaseName();

    protected abstract boolean manualTuneIsPossible();

    protected abstract boolean autoTuneIsPossible();

    protected abstract void init();

    protected abstract boolean manualTuned();

    protected abstract boolean autoTuned();

    protected abstract void reportResults();
}
