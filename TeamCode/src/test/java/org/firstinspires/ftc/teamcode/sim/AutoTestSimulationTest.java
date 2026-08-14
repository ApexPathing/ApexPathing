package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.junit.Test;

import java.lang.reflect.Method;
import java.io.File;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import core.FollowerConstants;
import core.ApexStorage;
import controllers.PDSController.PDSCoefficients;
import paths.movements.Path;

public class AutoTestSimulationTest {
    @Test(timeout = 90_000L)
    public void autoTestCompletesEveryMovement() throws Exception {
        configureStableConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        AutoTest auto = new AutoTest();
        auto.hardwareMap = hardware.hardwareMap;
        auto.telemetry = telemetry;
        auto.gamepad1 = new Gamepad();
        auto.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(auto, () -> { });
        try {
            pump(session, auto, telemetry, hardware, 100);
            long initializationDeadline = System.nanoTime() + 10_000_000_000L;
            while (auto.getOutboundPath() == null &&
                    System.nanoTime() < initializationDeadline) {
                pump(session, auto, telemetry, hardware, 20);
            }
            Path outbound = auto.getOutboundPath();
            assertTrue("Auto Test did not finish building its outbound path", outbound != null);
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 70_000_000_000L;
            while (!latest(frames).contains("Current check COMPLETE") &&
                    !latest(frames).contains("Current check FAILED")) {
                if (System.nanoTime() >= deadline) { break; }
                pump(session, auto, telemetry, hardware, 20);
            }

            String frame = latest(frames);
            assertFalse("Auto Test failed after its outbound path:\n" + frame,
                    frame.contains("Current check FAILED"));
            assertTrue("Auto Test did not complete every movement:\n" + frame,
                    frame.contains("Current check COMPLETE"));
            if (outbound.isProfiled()) {
                File velocityCsv = new File(auto.getOutboundVelocityCsvPath());
                assertTrue("Profiled outbound velocity CSV was not created: path=" +
                                auto.getOutboundVelocityCsvPath() + ", error=" +
                                auto.getOutboundVelocityCsvError(),
                        velocityCsv.isFile() && velocityCsv.length() > 0L);
            }
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(SimLinearOpModeBridge.Session session, AutoTest auto,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        long lastUpdate = System.nanoTime();
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0.001, Math.min(0.05, (now - lastUpdate) * 1e-9));
            lastUpdate = now;
            while (remaining > 1e-9) {
                double dt = Math.min(0.005, remaining);
                stepPhysics(hardware, dt);
                remaining -= dt;
            }
            SimLinearOpModeBridge.eventLoopIteration(session, auto.gamepad1, auto.gamepad2);
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
        Method kinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        kinematics.setAccessible(true);
        MotionVector robotVelocity = (MotionVector) kinematics.invoke(
                hardware.drivetrain, (Object) wheelVelocities);
        hardware.drivetrain.velocity = robotVelocity.toFieldFrame(
                hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, dt);
    }

    private static String latest(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void configureStableConstants() {
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
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
}
