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
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import core.FollowerConstants;

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

    @Test
    public void velocityFeedbackPhaseCanBeSelectedAndDisplayed() throws Exception {
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
            int exitFrameStart = frames.size();
            tuner.gamepad1.right_stick_button = true;
            pump(session, tuner, telemetry, 1650);
            tuner.gamepad1.right_stick_button = false;
            pump(session, tuner, telemetry, 50);

            assertTrue("Holding the displayed exit control did not disable debug mode",
                    frames.subList(Math.min(exitFrameStart, frames.size()), frames.size()).stream()
                            .anyMatch(frame -> frame.contains("E-STOP") &&
                                    !frame.contains("DEBUG MODE")));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    @Test
    public void backButtonEmergencyStopsFromAnActiveTunerScreen() throws Exception {
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
                tuner, () -> stopRequested.set(true));
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

            tuner.gamepad1.back = true;
            pump(session, tuner, telemetry, 50);
            tuner.gamepad1.back = false;
            pump(session, tuner, telemetry, 50);

            assertTrue("Back did not request an immediate OpMode stop", stopRequested.get());
            assertTrue("Emergency-stop confirmation was not shown",
                    frames.stream().anyMatch(frame ->
                            frame.contains("EMERGENCY STOP ACTIVATED")));
            assertFalse("Emergency stop left drivetrain power active",
                    anyDriveMotorPowered(hardware));
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

    private static void markAllPhasesCompleteForSelectionTest() {
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs.kP = 1.0;
        constants.angularKA = 1.0;
        constants.translationalCoeffs.kP = 1.0;
        constants.angularKV = 1.0;
        constants.translationalKV = 1.0;
        constants.translationalKA = 1.0;
        constants.kCentripetal = 1.0;
        constants.velocityFeedbackGain = 1.0;
        constants.angularVelocityFeedbackGain = 1.0;
    }

    private static boolean anyDriveMotorPowered(ApexSimulation.Hardware hardware) {
        for (String name : hardware.drivetrain.motorNames) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class, name);
            if (Math.abs(motor.getPower()) > 1e-9) { return true; }
        }
        return false;
    }

}
