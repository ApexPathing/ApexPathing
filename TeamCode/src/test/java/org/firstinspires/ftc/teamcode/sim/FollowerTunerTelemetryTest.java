package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;

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
}
