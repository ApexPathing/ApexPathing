package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.codeblooded.ftcodesim.input.Keybinds;
import org.codeblooded.ftcodesim.input.Keys;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.junit.Test;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import core.FollowerConstants;
import core.ApexStorage;
import controllers.PDSController.PDSCoefficients;
import tuning.PDSRoutine;

public class FollowerTunerTelemetryTest {
    @Test
    public void emitsPhaseSelectorTelemetryDuringInit() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(10);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        AtomicBoolean stopRequested = new AtomicBoolean();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(
                tuner,
                () -> stopRequested.set(true)
        );
        try {
            long deadline = System.nanoTime() + 2_000_000_000L;
            while (frames.isEmpty() && !stopRequested.get() && System.nanoTime() < deadline) {
                SimLinearOpModeBridge.eventLoopIteration(session, tuner.gamepad1, tuner.gamepad2);
                Thread.sleep(10);
            }
        } finally {
            SimLinearOpModeBridge.stop(session);
        }

        assertFalse("FollowerTuner did not emit telemetry during Init", frames.isEmpty());
    }

    @Test
    public void respondsToSelectionAndStartInputs() throws Exception {
        markAllPhasesCompleteForSelectionTest();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        AtomicBoolean stopRequested = new AtomicBoolean();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(
                tuner,
                () -> stopRequested.set(true)
        );
        try {
            pump(session, tuner, telemetry, 100);

            for (int i = 0; i < 6 && frames.stream().noneMatch(frame ->
                    frame.contains("HEADING <")); i++) {
                tuner.gamepad1.dpad_up = true;
                pump(session, tuner, telemetry, 30);
                tuner.gamepad1.dpad_up = false;
                pump(session, tuner, telemetry, 30);
            }

            tuner.gamepad1.dpad_down = true;
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.dpad_down = false;
            pump(session, tuner, telemetry, 100);

            assertTrue(
                    "D-pad Down did not highlight the next tuning phase",
                    frames.stream().anyMatch(frame -> frame.contains("LIMITS <"))
            );

            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.b = false;
            pump(session, tuner, telemetry, 100);

            assertTrue(
                    "FollowerTuner did not leave the phase selector after B",
                    frames.stream().anyMatch(frame -> frame.contains("Press Start to run the tuner."))
            );

            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, 200);

            assertTrue(
                    "FollowerTuner did not show phase telemetry after Start",
                    frames.stream().anyMatch(frame -> frame.contains("phase initialized"))
            );
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test
    public void startKeepsOptionsOpenUntilPhaseIsExplicitlySelected() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        AtomicBoolean stopRequested = new AtomicBoolean();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(
                tuner,
                () -> stopRequested.set(true)
        );
        try {
            pump(session, tuner, telemetry, 100);
            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, 200);

            assertTrue(
                    "Starting before selection did not keep the options menu visible",
                    frames.stream().anyMatch(frame -> frame.contains("choose a phase"))
            );
            assertFalse(
                    "Start silently accepted a highlighted phase",
                    frames.stream().anyMatch(frame -> frame.contains("phase initialized"))
            );

            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.b = false;
            pump(session, tuner, telemetry, 150);

            assertTrue(
                    "Explicit B selection did not open the selected phase",
                    frames.stream().anyMatch(frame -> frame.contains("phase initialized"))
            );
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test
    public void headingAutomaticModeShowsLiveProgressAfterA() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, 100);

            // Saved constants can make another phase the initial selection. Navigate explicitly
            // so this regression remains independent of the developer's simulator data.
            for (int i = 0; i < 6 && frames.stream().noneMatch(frame ->
                    frame.contains("HEADING <")); i++) {
                tuner.gamepad1.dpad_up = true;
                pump(session, tuner, telemetry, 30);
                tuner.gamepad1.dpad_up = false;
                pump(session, tuner, telemetry, 30);
            }
            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.b = false;
            pump(session, tuner, telemetry, 30);

            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, 100);

            Keybinds stockKeybinds = ApexSimulation.createConfig().gamepad1Keybinds;
            tuner.gamepad1.fromByteArray(stockKeybinds.getByteArray(
                    Collections.singleton(Keys.SEMICOLON)
            ));
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.fromByteArray(stockKeybinds.getByteArray(Collections.emptySet()));
            pump(session, tuner, telemetry, 100);

            assertTrue(
                    "Heading automatic tuner did not replace the selector with progress telemetry",
                    frames.stream().anyMatch(frame ->
                            frame.contains("Automatic heading tuning in progress"))
            );
            assertTrue("Normal telemetry did not explain what the robot was doing",
                    frames.stream().anyMatch(frame ->
                            frame.contains("Robot is finding the minimum power needed to turn")));
            assertFalse("Normal telemetry exposed debug-only search values",
                    frames.stream().anyMatch(frame -> frame.contains("Static guess")));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test(timeout = 90_000L)
    public void automaticHeadingOptimizationWaitsForAlternatingOperatorTests() throws Exception {
        int savedTrialRepeats = PDSRoutine.PD_TRIAL_REPEATS;
        int savedMaxIterations = PDSRoutine.PD_MAX_ITERATIONS;
        PDSRoutine.PD_TRIAL_REPEATS = 1;
        PDSRoutine.PD_MAX_ITERATIONS = 2;
        configureStableFollowerConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);
            for (int i = 0; i < 6 && !latestFrameContains(frames, "HEADING <"); i++) {
                tuner.gamepad1.dpad_up = true;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
                tuner.gamepad1.dpad_up = false;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            }
            assertTrue("Heading was not available in the phase selector",
                    latestFrameContains(frames, "HEADING <"));

            tuner.gamepad1.b = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.b = false;
            SimLinearOpModeBridge.start(session);
            long modeDeadline = System.nanoTime() + 3_000_000_000L;
            while (!latestFrameContains(frames, "Heading Controller phase initialized") &&
                    System.nanoTime() < modeDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Heading mode selector did not initialize",
                    latestFrameContains(frames, "Heading Controller phase initialized"));

            tuner.gamepad1.a = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.a = false;

            long checkDeadline = System.nanoTime() + 65_000_000_000L;
            while (!latestFrameContains(frames, "Ready for operator check") &&
                    System.nanoTime() < checkDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Automatic heading optimization did not reach the operator check. " +
                            "Latest telemetry:\n" + latestFrame(frames),
                    latestFrameContains(frames, "Ready for operator check"));

            pumpWithPhysics(session, tuner, telemetry, hardware, 250);
            assertFalse("Robot moved before the operator requested a response test",
                    anyDriveMotorPowered(hardware));

            tuner.gamepad1.x = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.x = false;
            long firstStartDeadline = System.nanoTime() + 1_000_000_000L;
            while (!latestFrameContains(frames, "turning to the requested test target") &&
                    System.nanoTime() < firstStartDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("X did not start the first operator-requested test",
                    latestFrameContains(frames, "turning to the requested test target"));

            long firstTestDeadline = System.nanoTime() + 10_000_000_000L;
            while (!latestFrameContains(frames, "Test complete") &&
                    System.nanoTime() < firstTestDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("First operator-requested heading test did not complete",
                    latestFrameContains(frames, "Test complete"));
            assertTrue("First heading test did not use the positive target",
                    hardware.drivetrain.position.theta > 0.0);
            double firstTestHeading = hardware.drivetrain.position.theta;

            tuner.gamepad1.x = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.x = false;
            long secondStartDeadline = System.nanoTime() + 1_000_000_000L;
            while (!latestFrameContains(frames, "turning to the requested test target") &&
                    System.nanoTime() < secondStartDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("X did not start the alternating return test",
                    latestFrameContains(frames, "turning to the requested test target"));

            long secondTestDeadline = System.nanoTime() + 10_000_000_000L;
            while (!latestFrameContains(frames, "Test complete") &&
                    System.nanoTime() < secondTestDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Alternating heading test did not complete",
                    latestFrameContains(frames, "Test complete"));
            assertTrue("Second heading test did not return toward its starting heading",
                    Math.abs(hardware.drivetrain.position.theta) < Math.abs(firstTestHeading));

            tuner.gamepad1.a = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.a = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);

            assertTrue("A did not accept the operator-checked gains. Latest " +
                            "telemetry:\n" + latestFrame(frames),
                    latestFrameContains(frames,
                            "Heading Controller phase complete with results"));
        } finally {
            SimLinearOpModeBridge.stop(session);
            PDSRoutine.PD_TRIAL_REPEATS = savedTrialRepeats;
            PDSRoutine.PD_MAX_ITERATIONS = savedMaxIterations;
        }
    }

    @Test
    public void manualVelocityFeedbackWaitsForStartAndRestartsSafely() throws Exception {
        markAllPhasesCompleteForSelectionTest();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, 100);

            for (int i = 0; i < 6 && frames.stream().noneMatch(frame ->
                    frame.contains("VELOCITY FEEDBACK <")); i++) {
                tuner.gamepad1.dpad_down = true;
                pump(session, tuner, telemetry, 30);
                tuner.gamepad1.dpad_down = false;
                pump(session, tuner, telemetry, 30);
            }
            assertTrue("Velocity Feedback was not available in the phase selector",
                    frames.stream().anyMatch(frame -> frame.contains("VELOCITY FEEDBACK <")));

            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.b = false;
            pump(session, tuner, telemetry, 30);
            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, 150);

            assertTrue("Velocity Feedback did not initialize after selection",
                    frames.stream().anyMatch(frame ->
                            frame.contains("Velocity Feedback phase initialized")));

            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.b = false;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.a = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.a = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);

            assertTrue("Manual velocity selection was not marked inline",
                    latestFrameContains(frames, "-> Translation feedback"));
            assertFalse("Normal velocity telemetry still shows verbose diagnostics",
                    latestFrame(frames).contains("Test state") ||
                            latestFrame(frames).contains("Usable samples") ||
                            latestFrame(frames).contains("Response CSV"));
            assertFalse("Manual velocity feedback moved before X",
                    anyDriveMotorPowered(hardware));

            tuner.gamepad1.x = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.x = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 120);
            assertTrue("X did not start the manual velocity test",
                    anyDriveMotorPowered(hardware));

            tuner.gamepad1.dpad_up = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.dpad_up = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 80);
            assertTrue("Gain edit did not safely restart the active test",
                    latestFrameContains(frames, "-> Translation feedback"));

            tuner.gamepad1.right_bumper = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.right_bumper = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 80);
            assertFalse("Changing feedback axes did not stop the active movement",
                    anyDriveMotorPowered(hardware));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test(timeout = 35_000L)
    public void manualDrivePdsRunsStraightOutAndBackWithCompactTelemetry() throws Exception {
        configureStableFollowerConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);
            for (int i = 0; i < 6 && !latestFrameContains(frames, "DRIVE <"); i++) {
                tuner.gamepad1.dpad_down = true;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
                tuner.gamepad1.dpad_down = false;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            }
            assertTrue("Drive was not available in the phase selector",
                    latestFrameContains(frames, "DRIVE <"));

            tuner.gamepad1.b = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.b = false;
            SimLinearOpModeBridge.start(session);
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);

            tuner.gamepad1.b = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.b = false;
            tuner.gamepad1.a = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.a = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);

            String normalFrame = latestFrame(frames);
            assertTrue("Drive coefficient selection was not marked inline",
                    normalFrame.contains("-> Drive P"));
            assertFalse("Normal drive telemetry still shows verbose diagnostics",
                    normalFrame.contains("Selected") || normalFrame.contains("Overshoot") ||
                            normalFrame.contains("Settling time") ||
                            normalFrame.contains("Response CSV"));

            tuner.gamepad1.x = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.x = false;
            long outboundDeadline = System.nanoTime() + 8_000_000_000L;
            while ((hardware.drivetrain.position.x < ApexSimulation.FIELD_CENTER_INCHES + 20.0 ||
                    anyDriveMotorPowered(hardware)) &&
                    System.nanoTime() < outboundDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Manual drive did not reach its 24-inch outbound target",
                    hardware.drivetrain.position.x > ApexSimulation.FIELD_CENTER_INCHES + 20.0);
            assertTrue("Manual drive path changed heading on the outbound leg",
                    Math.abs(hardware.drivetrain.position.theta) < Math.toRadians(3.0));

            tuner.gamepad1.x = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);
            tuner.gamepad1.x = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);
            long returnDeadline = System.nanoTime() + 8_000_000_000L;
            while ((hardware.drivetrain.position.x > ApexSimulation.FIELD_CENTER_INCHES + 4.0 ||
                    anyDriveMotorPowered(hardware)) &&
                    System.nanoTime() < returnDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Manual drive did not return toward its starting position; x=" +
                            hardware.drivetrain.position.x + ", powered=" +
                            anyDriveMotorPowered(hardware) + ", telemetry=\n" + latestFrame(frames),
                    hardware.drivetrain.position.x < ApexSimulation.FIELD_CENTER_INCHES + 4.0);
            assertTrue("Manual reverse path changed heading",
                    Math.abs(hardware.drivetrain.position.theta) < Math.toRadians(3.0));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test
    public void debugEntryIsHiddenAndHoldCanEnableAndDisableIt() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, 100);
            assertFalse("Normal menu revealed the hidden debug entry control",
                    frames.stream().anyMatch(frame -> frame.contains("Hold Left Stick Button")));

            tuner.gamepad1.left_stick_button = true;
            pump(session, tuner, telemetry, 1650);
            assertTrue("Holding the hidden menu control did not enable debug mode",
                    frames.stream().anyMatch(frame -> frame.contains("DEBUG MODE")));
            assertTrue("Debug mode did not explain how to exit",
                    frames.stream().anyMatch(frame ->
                            frame.contains("Hold Right Stick Button") &&
                                    frame.contains("exit debug mode")));

            tuner.gamepad1.left_stick_button = false;
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.right_stick_button = true;
            pump(session, tuner, telemetry, 1650);
            tuner.gamepad1.right_stick_button = false;
            pump(session, tuner, telemetry, 50);

            assertFalse("Holding the displayed exit control did not disable debug mode",
                    frames.get(frames.size() - 1).contains("DEBUG MODE"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test
    public void stopButtonStopsDrivetrainFromAnActiveTunerScreen() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        boolean stopped = false;
        try {
            pump(session, tuner, telemetry, 100);
            tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.b = false;
            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, 100);

            tuner.gamepad1.a = true;
            pump(session, tuner, telemetry, 30);
            tuner.gamepad1.a = false;
            long motionDeadline = System.nanoTime() + 1_000_000_000L;
            while (!anyDriveMotorPowered(hardware) && System.nanoTime() < motionDeadline) {
                pump(session, tuner, telemetry, 20);
            }
            assertTrue("Test did not reach an actively powered tuner state",
                    anyDriveMotorPowered(hardware));

            SimLinearOpModeBridge.stop(session);
            stopped = true;
            assertFalse("Stop button left drivetrain power active", anyDriveMotorPowered(hardware));
        } finally {
            if (!stopped) { SimLinearOpModeBridge.stop(session); }
        }
    }

    @Test(timeout = 60_000L)
    public void automaticFeedforwardCharacterizationPassesWithSimMotorModel() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        markAllPhasesCompleteForSelectionTest();
        // This test enters feedforward directly instead of running the prerequisite PDS phases.
        // Match their static-gain result to the simulated drivetrain's 0.45 breakaway power.
        FollowerConstants.getInstance().angularCoeffs.kS = 0.45;
        FollowerConstants.getInstance().translationalCoeffs.kS = 0.45;
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pumpWithPhysics(session, tuner, telemetry, hardware, 100);
            for (int i = 0; i < 6 && !latestFrameContains(frames, "FEEDFORWARD <"); i++) {
                tuner.gamepad1.dpad_down = true;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
                tuner.gamepad1.dpad_down = false;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            }
            assertTrue("Feedforward was not available in the phase selector",
                    latestFrameContains(frames, "FEEDFORWARD <"));

            tuner.gamepad1.b = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.b = false;
            SimLinearOpModeBridge.start(session);
            long modeDeadline = System.nanoTime() + 3_000_000_000L;
            while (!latestFrameContains(frames, "Feedforward Refinement phase initialized") &&
                    System.nanoTime() < modeDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Feedforward mode selector did not initialize",
                    latestFrameContains(frames, "Feedforward Refinement phase initialized"));

            tuner.gamepad1.a = true;
            pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.a = false;
            pumpWithPhysics(session, tuner, telemetry, hardware, 50);

            for (int run = 1; run <= 8; run++) {
                String prompt = "Feedforward characterization " + run + " / 8";
                long promptDeadline = System.nanoTime() + 6_000_000_000L;
                while (!latestFrameContains(frames, prompt) &&
                        System.nanoTime() < promptDeadline) {
                    pumpWithPhysics(session, tuner, telemetry, hardware, 20);
                }
                assertTrue("Timed out waiting for " + prompt, latestFrameContains(frames, prompt));

                long stationaryDeadline = System.nanoTime() + 3_000_000_000L;
                while (!latestFrameContains(frames, "Robot is stationary and ready.") &&
                        System.nanoTime() < stationaryDeadline) {
                    pumpWithPhysics(session, tuner, telemetry, hardware, 20);
                }
                assertTrue("Robot did not settle before " + prompt,
                        latestFrameContains(frames, "Robot is stationary and ready."));

                tuner.gamepad1.a = true;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
                tuner.gamepad1.a = false;
                pumpWithPhysics(session, tuner, telemetry, hardware, 30);
            }

            long resultDeadline = System.nanoTime() + 4_000_000_000L;
            while (!latestFrameContains(frames, "Validation Angular PASSED; Translation PASSED") &&
                    !latestFrameContains(frames, "did not pass validation") &&
                    System.nanoTime() < resultDeadline) {
                pumpWithPhysics(session, tuner, telemetry, hardware, 20);
            }

            assertTrue("Simulated characterization did not pass. Latest telemetry:\n" +
                    latestFrame(frames),
                    latestFrameContains(frames,
                            "Validation Angular PASSED; Translation PASSED"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(
            SimLinearOpModeBridge.Session session,
            FollowerTuner tuner,
            ApexSimTelemetry telemetry,
            long milliseconds
    ) throws InterruptedException {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        while (System.nanoTime() < deadline) {
            SimLinearOpModeBridge.eventLoopIteration(
                    session,
                    tuner.gamepad1,
                    tuner.gamepad2
            );
            telemetry.update();
            Thread.sleep(5);
        }
    }

    private static void pumpWithPhysics(
            SimLinearOpModeBridge.Session session,
            FollowerTuner tuner,
            ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware,
            long milliseconds
    ) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        long lastPhysicsUpdate = System.nanoTime();
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remainingDt = Math.max(0.001,
                    Math.min(0.05, (now - lastPhysicsUpdate) * 1e-9));
            lastPhysicsUpdate = now;
            while (remainingDt > 1e-9) {
                double substep = Math.min(0.005, remainingDt);
                stepPhysics(hardware, substep);
                remainingDt -= substep;
            }
            SimLinearOpModeBridge.eventLoopIteration(
                    session,
                    tuner.gamepad1,
                    tuner.gamepad2
            );
            telemetry.update();
            Thread.sleep(5);
        }
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double dt) throws Exception {
        double[] wheelVelocities = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < hardware.drivetrain.motorNames.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class, hardware.drivetrain.motorNames[i]);
            motor.update(dt);
            wheelVelocities[i] = motor.getVelocity();
        }

        java.lang.reflect.Method forwardKinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        forwardKinematics.setAccessible(true);
        MotionVector robotVelocity = (MotionVector) forwardKinematics.invoke(
                hardware.drivetrain, (Object) wheelVelocities);
        hardware.drivetrain.velocity = robotVelocity.toFieldFrame(
                hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, dt);
    }

    private static boolean latestFrameContains(List<String> frames, String text) {
        return latestFrame(frames).contains(text);
    }

    private static String latestFrame(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void markAllPhasesCompleteForSelectionTest() {
        configureStableFollowerConstants();
    }

    private static void configureStableFollowerConstants() {
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
        // Keep the shared singleton deterministic and conservative enough for the bounded
        // closed-loop tuner trials. Production gains can be much higher once characterized.
        constants.angularCoeffs = new PDSCoefficients(0.80, 0.10, 0.23);
        constants.translationalCoeffs = new PDSCoefficients(0.12, 0.03, 0.23);
        constants.angularKV = 0.066;
        constants.angularKA = 0.043;
        constants.translationalKV = 0.0071;
        constants.translationalKA = 0.0047;
        constants.kCentripetal = 0.0061;
        constants.velocityFeedbackGain = 0.059;
        constants.angularVelocityFeedbackGain = 0.25;
        constants.forwardVelLimitIn = 64.7;
        constants.forwardAccelLimitIn = 105.7;
        constants.strafeVelLimitIn = 53.6;
        constants.strafeAccelLimitIn = 84.9;
        constants.angularVelLimitRad = 6.96;
        constants.angularAccelLimitRad = 12.0;
    }

    private static boolean anyDriveMotorPowered(ApexSimulation.Hardware hardware) {
        for (String name : hardware.drivetrain.motorNames) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class, name);
            if (Math.abs(motor.getPower()) > 1e-9) { return true; }
        }
        return false;
    }

}
