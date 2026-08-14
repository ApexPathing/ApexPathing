package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import geometry.Pose;

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
            context.updateDebugMode(false);
            context.getTelemetry().clearAll();
            context.addInterfaceHeader();

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

            // A LinearOpMode can otherwise spin much faster than the hardware/localizer can
            // provide new samples. Besides wasting CPU, differentiating the same pose repeatedly
            // and then one discrete update creates enormous acceleration spikes (especially in
            // FTCodeSim, whose physics advances every 20 ms). Use a deterministic 50 Hz sampling
            // cadence for every tuner phase.
            opMode.sleep(20);
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
        showPreRunInstructions();
        context.getTelemetry().addLine("Press A to run this phase.");
        context.getTelemetry().update();
    }

    /** Adds phase-specific positioning or safety guidance before the operator starts motion. */
    protected void showPreRunInstructions() { }

    private void showResults() {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with results:");
        reportResults();
        context.getTelemetry().addLine("Press B to continue.");
        context.getTelemetry().update();
    }

    /** Displays a compact editable value list without spending a separate line on selection. */
    protected void addTunableValue(String label, double value, boolean selected) {
        context.getTelemetry().addLine((selected ? "-> " : "   ") + label + ": " +
                context.formatNumber(value));
    }

    protected String number(double value) { return context.formatNumber(value); }

    /**
     * Places a simulated movement test at its phase-specific staging pose. On hardware, resetting
     * odometry cannot move the physical robot, so positioning remains the operator's responsibility.
     */
    protected void positionRobotForSimulation(Pose pose) {
        context.positionRobotForSimulation(pose);
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
