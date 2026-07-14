package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.HeadingPhase;
import tuning.TranslationalPhase;
import tuning.VelocityFeedforwardPhase;
import tuning.MovementLimitsPhase;

import core.Follower;
import geometry.Pose;

/**
 * Unified follower tuning class for Apex Pathing
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    private final TunerContext context = new TunerContext(this);

    private String[] phaseSelectorStatuses;
    private int selectedPhaseIndex = 0;
    private TuningPhase runningPhase = null;

    enum Phase { HEADING_PDS, TRANSLATIONAL_PDS, VELOCITY_FEEDFORWARD, MOVEMENT_LIMITS }
    private static final Phase[] phases = {
            Phase.HEADING_PDS,
            Phase.TRANSLATIONAL_PDS,
            Phase.VELOCITY_FEEDFORWARD,
            Phase.MOVEMENT_LIMITS
    };

    @Override
    public void runOpMode() {
        context.setFollower(new Follower(new Constants(), hardwareMap));
        resetPhaseSelectionData(); // Initialize the phase selector statuses

        while (opModeInInit() && runningPhase == null) {
            TuningPhase selectedPhase = phaseSelector(); // Will remain null until a phase is selected
            if (selectedPhase != null) {
                runningPhase = selectedPhase;
            }
        }

        if (runningPhase != null) {
            context.getFollower().setPose(Pose.zero());
            telemetry.addLine("Press the start button to run the tuner");
            telemetry.addLine("Make sure you have adequate space to run the robot safely!");
            telemetry.update();
        } else {
            requestOpModeStop(); // The OpMode was started without a phase, so we stop it
        }

        waitForStart();

        while (opModeIsActive()) {
            if (runningPhase == null) {
                continue; // OpMode stop was requested already, do nothing
            }
            context.getFollower().update();
            boolean complete = runningPhase.update(gamepad1.aWasPressed(), gamepad1.bWasPressed());
            if (complete) {
                runningPhase = null;
                context.constants.drivetrainType = context.getFollower().getDrivetrain()
                        .getDrivetrainType();
                context.saveConstants();
                requestOpModeStop();
            }
        }
    }

    private void resetPhaseSelectionData() {
        boolean headingIsTuned = context.constants.headingCoeffs.kP != 0.0;
        boolean translationalIsTuned = context.constants.translationalCoeffs.kP != 0.0;
        boolean velocityFFIsTuned = context.constants.translationalKV != 0.0;
        boolean movementLimitsIsTuned = context.constants.strafeAccelLimitIn != 0.0;

        String headingStatus = headingIsTuned ? "[✓]" : "[ ]"; // Heading is always available to tune first.
        String transStatus = translationalIsTuned ? "[✓]" : (headingIsTuned ? "[ ]" : "[X]");
        String velStatus = velocityFFIsTuned ? "[✓]" : (translationalIsTuned ? "[ ]" : "[X]");
        String movementStatus = movementLimitsIsTuned ? "[✓]" : (velocityFFIsTuned ? "[ ]" : "[X]");
        this.phaseSelectorStatuses = new String[]{
                headingStatus, transStatus, velStatus, movementStatus
        };

        this.selectedPhaseIndex = 0;
        for (int i = 0; i < phaseSelectorStatuses.length; i++) {
            if (phaseSelectorStatuses[i].equals("[ ]")) {
                this.selectedPhaseIndex = i;
                break;
            }
        }
    }

    private TuningPhase phaseSelector() {
        telemetry.addLine(
                "The Apex Pathing tuners are listed in order of execution below."
        );
        telemetry.addLine(
                "[✓] = Already Tuned (you can still select to retune)," +
                        "[X] = Not available to tune (incomplete tuners before it)," +
                        "[ ] = Next tuner to run. The cursor ('<') is here by default."
        );
        telemetry.addLine("Use the DPad Up and Down buttons to cycle through phases, " +
                "then press B to open the selected phase.");
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = (i == selectedPhaseIndex) ? " <" : "";
            telemetry.addLine(phaseSelectorStatuses[i] + " " +
                    phases[i].toString().replace("_", " ") + cursor);
        }
        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            // Don't allow selection of unavailable phases
            while (phaseSelectorStatuses[selectedPhaseIndex].equals("[X]")) {
                selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            while (phaseSelectorStatuses[selectedPhaseIndex].equals("[X]")) {
                selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            }
        } else if (gamepad1.bWasPressed()) {
            Phase selectedPhase = phases[selectedPhaseIndex];
            switch (selectedPhase) {
                case HEADING_PDS:
                    return new HeadingPhase(context);
                case TRANSLATIONAL_PDS:
                    return new TranslationalPhase(context);
                case VELOCITY_FEEDFORWARD:
                    return new VelocityFeedforwardPhase(context);
                case MOVEMENT_LIMITS:
                    return new MovementLimitsPhase(context);
            }
        }

        return null; // Phase hasn't been selected yet
    }
}
