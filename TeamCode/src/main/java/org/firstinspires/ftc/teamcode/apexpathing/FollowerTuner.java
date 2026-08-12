package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Predicate;

import core.Follower;
import core.FollowerConstants;
import geometry.Pose;
import tuning.CentripetalPhase;
import tuning.DrivePhase;
import tuning.FeedforwardTuner;
import tuning.HeadingPhase;
import tuning.LimitsPhase;
import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.VelocityFeedbackPhase;

/**
 * This OpMode is used to tune the Apex Pathing Follower. It allows the user to select a tuning
 * phase at which to begin, then runs each remaining phase in order and saves after every phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing")
public class FollowerTuner extends LinearOpMode {
    /** Allows the desktop simulator to exercise phases without saved prerequisite constants. */
    public static final String UNLOCK_PHASES_PROPERTY = "apex.simulation.unlockTunerPhases";

    /**
     * Completion is determined by whether the last saved value of the phase's constants is non-zero
     * Tuners are ran in the order of the enum ordinals
     */
    enum Phase {
        HEADING(HeadingPhase.class, (FollowerConstants constants) ->
                constants.angularCoeffs.kP != 0.0),
        LIMITS(LimitsPhase.class, (FollowerConstants constants) ->
                constants.angularKA != 0.0),
        DRIVE(DrivePhase.class, (FollowerConstants constants) ->
                constants.translationalCoeffs.kP != 0.0),
        FEEDFORWARD(FeedforwardTuner.class, (FollowerConstants constants) ->
                constants.angularKV != 0.0 && constants.angularKA != 0.0 &&
                        constants.translationalKV != 0.0 && constants.translationalKA != 0.0),
        CENTRIPETAL(CentripetalPhase.class, (FollowerConstants constants) ->
                constants.kCentripetal != 0.0),
        VELOCITY_FEEDBACK(VelocityFeedbackPhase.class, (FollowerConstants constants) ->
                constants.angularVelocityFeedbackGain != 0.0);

        final Class<? extends TuningPhase> phaseClass;
        final Predicate<FollowerConstants> isTunedPredicate;
        boolean tuned;

        Phase(Class<? extends TuningPhase> phaseClass,
              Predicate<FollowerConstants> isTunedPredicate) {
            this.phaseClass = phaseClass;
            this.isTunedPredicate = isTunedPredicate;
        }

        void updateTunedStatus(FollowerConstants constants) {
            tuned = isTunedPredicate.test(constants);
        }
    }

    private static final Phase[] phases = Phase.values();
    private static final int phaseAmount = phases.length;

    private TunerContext context;
    private Phase selectedPhaseOrdinal;
    private TuningPhase phase;
    private boolean isPhaseSelected = false;

    @Override
    public void runOpMode() {
        context = new TunerContext(this);
        context.setFollower(new Follower(new Constants(), hardwareMap, true));
        context.constants.drivetrainType = context.getFollower().getDrivetrain().getDrivetrainType();
        
        for (Phase phase : phases) { phase.updateTunedStatus(context.constants); }
        selectFirstIncompletePhase();

        while (opModeInInit() && !isPhaseSelected) { isPhaseSelected = phaseSelector(); }

        if (isPhaseSelected) {
            telemetry.addLine("Press Start to run the tuner.");
            telemetry.addLine("Make sure the robot has enough space.");
            telemetry.update();
        }

        waitForStart();

        // Starting the OpMode must never silently accept a highlighted option. If Start was
        // pressed before a phase was selected, keep presenting the same menu while RUNNING until
        // the user explicitly confirms a phase.
        while (opModeIsActive() && !isPhaseSelected) {
            isPhaseSelected = phaseSelector();
            sleep(20);
        }
        if (!opModeIsActive()) {
            context.getFollower().stop();
            return;
        }

        context.getFollower().setPose(Pose.zero());

        while (opModeIsActive()) {
            if (phase.run(this)) { // Returns true if the phase is complete
                context.saveConstants();
                selectedPhaseOrdinal.updateTunedStatus(context.constants);

                Phase nextPhase = nextPhase(selectedPhaseOrdinal);
                if (nextPhase == null) {
                    finishTuningWorkflow();
                    break;
                }

                context.getFollower().stop();
                context.getFollower().enableControllers();
                context.getFollower().setPose(Pose.zero());
                selectedPhaseOrdinal = nextPhase;
                selectPhase();
            }
        }

        context.getFollower().stop();
    }

    /** Loops through phases before the given phase to check if they have been tuned or not. */
    private boolean phaseAvailable(Phase phase) {
        if (Boolean.getBoolean(UNLOCK_PHASES_PROPERTY)) { return true; }

        for (int i = 0; i < phase.ordinal(); i++) {
            if (!phases[i].tuned) {
                return false;
            }
        }
        return true;
    }

    private String phaseStatus(Phase phase) {
        if (phase.tuned) { return "[ ✓ ]"; }
        return phaseAvailable(phase) ? "[   ]" : "[ X ]";
    }

    private void selectFirstIncompletePhase() {
        selectedPhaseOrdinal = phases[0];
        for (int i = 0; i < phaseAmount; i++) {
            if (!phases[i].tuned && phaseAvailable(phases[i])) {
                selectedPhaseOrdinal = phases[i];
                return;
            }
        }
    }

    private boolean phaseSelector() {
        String selectControl = Boolean.getBoolean(UNLOCK_PHASES_PROPERTY)
                ? "B [left-bracket key]"
                : "B";
        telemetry.addLine("Use Dpad Up and Down to choose a phase, then press " +
                selectControl + " to select it.");
        telemetry.addLine();

        for (int i = 0; i < phaseAmount; i++) {
            String cursor = i == selectedPhaseOrdinal.ordinal() ? " <" : "";
            telemetry.addLine(phaseStatus(phases[i]) + " " +
                    phases[i].name().replace("_", " ") + cursor);
        }

        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            do {
                selectedPhaseOrdinal = phases[(selectedPhaseOrdinal.ordinal() - 1 + phaseAmount) % phaseAmount];
            } while (!phaseAvailable(selectedPhaseOrdinal));
        } else if (gamepad1.dpadDownWasPressed()) {
            do {
                selectedPhaseOrdinal = phases[(selectedPhaseOrdinal.ordinal() + 1) % phaseAmount];
            } while (!phaseAvailable(selectedPhaseOrdinal));
        } else if (gamepad1.bWasPressed()) {
            selectPhase();
            return true;
        }

        return false;
    }

    private void selectPhase() {
        try {
            phase = selectedPhaseOrdinal.phaseClass.getDeclaredConstructor(TunerContext.class)
                    .newInstance(context);
            isPhaseSelected = true;
        } catch (Exception e) {
            // This won't happen because the setup is correct, but Java requires the catch.
            throw new RuntimeException(e);
        }
    }

    static Phase nextPhase(Phase current) {
        int nextOrdinal = current.ordinal() + 1;
        return nextOrdinal < phaseAmount ? phases[nextOrdinal] : null;
    }

    private void finishTuningWorkflow() {
        telemetry.clearAll();
        telemetry.addLine("All follower tuning phases are complete.");

        // FTCodeSim does not move its Driver Station out of RUNNING when a LinearOpMode calls
        // requestOpModeStop(). Keep the final lifecycle alive until the red Stop button is used.
        if (Boolean.getBoolean(UNLOCK_PHASES_PROPERTY)) {
            telemetry.addLine("Press the red STOP button to finish this simulation.");
            telemetry.update();
            while (opModeIsActive()) { sleep(50); }
            return;
        }

        telemetry.update();
        requestOpModeStop();
    }
}
