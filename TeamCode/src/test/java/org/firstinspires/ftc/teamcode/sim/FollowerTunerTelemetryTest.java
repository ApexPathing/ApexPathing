package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.codeblooded.ftcodesim.input.Keybinds;
import org.codeblooded.ftcodesim.input.Keys;
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

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

}
