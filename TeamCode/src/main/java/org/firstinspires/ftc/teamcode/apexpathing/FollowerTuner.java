package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Predicate;

import core.Follower;
import core.FollowerConstants;
import geometry.Pose;
import tuning.CentripetalPhase;
import tuning.DrivePhase;
import tuning.HeadingPhase;
import tuning.LimitsPhase;
import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.VelocityFeedbackPhase;

/**
 * This OpMode is used to tune the Apex Pathing Follower. It allows the user to select a tuning
 * phase and run it, saving the constants when complete.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing")
public class FollowerTuner extends LinearOpMode {
    /**
     * Completion is determined by whether the last saved value of the phase's constants is non-zero
     * Tuners are ran in the order of the enum ordinals
     */
    enum Phase {
        HEADING(HeadingPhase.class, (FollowerConstants constants) ->
                constants.headingCoeffs.kP != 0.0),
        LIMITS(LimitsPhase.class, (FollowerConstants constants) ->
                constants.angular.KA != 0.0),
        DRIVE(DrivePhase.class, (FollowerConstants constants) ->
                constants.translationalCoeffs.kP != 0.0),
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

        telemetry.addLine("Press Start to run the tuner.");
        telemetry.addLine("Make sure the robot has enough space.");
        telemetry.update();

        waitForStart();
        context.getFollower().setPose(Pose.zero());

        while (opModeIsActive()) {
            if (phase.run(this)) { // Returns true if the phase is complete
                context.saveConstants();
                requestOpModeStop();
            }
        }

        context.getFollower().stop();
    }

    /** Loops through phases before the given phase to check if they have been tuned or not. */
    private boolean phaseAvailable(Phase phase) {
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
        telemetry.addLine("Use Dpad Up and Down to choose a phase, then press B to select it.");
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
            try {
                phase = selectedPhaseOrdinal.phaseClass.getDeclaredConstructor(TunerContext.class)
                        .newInstance(context);
            } catch (Exception e) {
                // This won't happen because the setup is correct, but Java requires the catch.
                throw new RuntimeException(e);
            }
            return true;
        }

        return false;
    }
}
