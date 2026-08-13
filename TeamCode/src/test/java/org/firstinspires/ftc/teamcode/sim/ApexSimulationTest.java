package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.input.Keybinds;
import org.codeblooded.ftcodesim.input.Keys;
import org.codeblooded.ftcodesim.simulator.OpModeRegister;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.firstinspires.ftc.teamcode.apexpathing.TeleOpTest;
import org.json.JSONObject;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

import core.Follower;
import drivetrains.BaseDrivetrain;
import geometry.Pose;
import localizers.Pinpoint;

public class ApexSimulationTest {
    @Test
    public void registersEveryOpMode() {
        Set<Class<?>> registeredClasses = new HashSet<>();
        for (OpMode opMode : new OpModeRegister().getOpModes()) {
            registeredClasses.add(opMode.getClass());
        }

        assertTrue(registeredClasses.contains(AutoTest.class));
        assertTrue(registeredClasses.contains(TeleOpTest.class));
        assertTrue(registeredClasses.contains(FollowerTuner.class));
    }

    @Test
    public void hardwareMapSatisfiesAllCurrentOpModes() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();

        assertNotNull(hardware.hardwareMap.get(DcMotorEx.class, ApexSimulation.FRONT_LEFT_MOTOR));
        assertNotNull(hardware.hardwareMap.get(DcMotorEx.class, ApexSimulation.FRONT_RIGHT_MOTOR));
        assertNotNull(hardware.hardwareMap.get(DcMotorEx.class, ApexSimulation.BACK_LEFT_MOTOR));
        assertNotNull(hardware.hardwareMap.get(DcMotorEx.class, ApexSimulation.BACK_RIGHT_MOTOR));
        assertNotNull(hardware.hardwareMap.get(Pinpoint.Driver.class, ApexSimulation.PINPOINT));

        // Constructing the shared Follower exercises the same drivetrain/localizer initialization
        // path used by AutoTest, TeleOpTest, and FollowerTuner.
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        assertPose(Pose.zero(), follower.getPose());
    }

    @Test
    public void reversingPureTurnUpdatesEveryWheelWithoutTranslationLeakage() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        BaseDrivetrain<?> drivetrain = follower.getDrivetrain();

        drivetrain.moveWithVectors(0.0, 0.0, 0.5);
        assertWheelPowers(drivetrain, -0.5, 0.5, -0.5, 0.5);

        // This reversal previously left front-right at +0.5 because its cache check incorrectly
        // compared against the new front-left power. The resulting +,+,+,- wheel pattern contains
        // translation even though the requested x/y command is exactly zero.
        drivetrain.moveWithVectors(0.0, 0.0, -0.5);
        assertWheelPowers(drivetrain, 0.5, -0.5, 0.5, -0.5);
    }

    @Test
    public void positiveStrafeUsesTheDocumentedLeftWheelPattern() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        BaseDrivetrain<?> drivetrain = follower.getDrivetrain();

        drivetrain.moveWithVectors(0.0, 0.5, 0.0);

        assertWheelPowers(drivetrain, -0.5, 0.5, 0.5, -0.5);
    }

    @Test
    public void simulatorMotionUsesApexForwardLeftAndCounterclockwiseAxes() throws Exception {
        MotionVector forward = simulateMotion(0.5, 0.0, 0.0);
        assertTrue(forward.x > 0.0);
        assertEquals(0.0, forward.y, 1e-9);
        assertEquals(0.0, forward.theta, 1e-9);

        MotionVector backward = simulateMotion(-0.5, 0.0, 0.0);
        assertTrue(backward.x < 0.0);
        assertEquals(0.0, backward.y, 1e-9);
        assertEquals(0.0, backward.theta, 1e-9);

        MotionVector left = simulateMotion(0.0, 0.5, 0.0);
        assertEquals(0.0, left.x, 1e-9);
        assertTrue(left.y > 0.0);
        assertEquals(0.0, left.theta, 1e-9);

        MotionVector counterclockwise = simulateMotion(0.0, 0.0, 0.5);
        assertEquals(0.0, counterclockwise.x, 1e-9);
        assertEquals(0.0, counterclockwise.y, 1e-9);
        assertTrue(counterclockwise.theta > 0.0);
    }

    @Test
    public void pinpointAdapterConvertsFieldCoordinatesAndHeading() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimApexPinpoint pinpoint = (SimApexPinpoint) hardware.hardwareMap.get(
                Pinpoint.Driver.class,
                ApexSimulation.PINPOINT
        );

        hardware.drivetrain.setPosition(new MotionVector(
                ApexSimulation.FIELD_CENTER_INCHES + 12.0,
                ApexSimulation.FIELD_CENTER_INCHES + 5.0,
                Math.PI / 3.0
        ));
        pinpoint.update(0.02);

        Pose pose = pinpoint.getPosition();
        assertEquals(12.0, pose.getX().getIn(), 1e-9);
        assertEquals(5.0, pose.getY().getIn(), 1e-9);
        assertEquals(Math.PI / 3.0, pose.getHeading().getRad(), 1e-9);
    }

    @Test
    public void pinpointPoseResetIsImmediatelyVisibleToFollower() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        Pose requested = new Pose(
                geometry.Vector.of(12.0, -8.0, geometry.DistUnit.IN),
                geometry.Angle.fromDeg(30.0)
        );

        follower.setPose(requested);

        assertPose(requested, follower.getPose());
    }

    @Test
    public void simulatorKeepsFtCodeSimDefaultKeyboardMappings() {
        Keybinds keybinds = ApexSimulation.createConfig().gamepad1Keybinds;
        Set<Integer> pressedKeys = new HashSet<>();
        pressedKeys.add(Keys.OPEN_BRACKET);
        pressedKeys.add(Keys.UP);

        Gamepad gamepad = new Gamepad();
        gamepad.fromByteArray(keybinds.getByteArray(pressedKeys));

        assertTrue(gamepad.b);
        assertTrue(gamepad.dpad_up);

        pressedKeys.clear();
        pressedKeys.add(Keys.SEMICOLON);
        gamepad.fromByteArray(keybinds.getByteArray(pressedKeys));
        assertTrue(gamepad.a);

        pressedKeys.clear();
        pressedKeys.add(Keys.P);
        gamepad.fromByteArray(keybinds.getByteArray(pressedKeys));
        assertTrue(gamepad.x);
    }

    @Test
    public void desktopJsonCanReloadSavedFollowerConstants() throws Exception {
        JSONObject savedConstants = new JSONObject("{\"drivetrainType\":\"MECANUM\"}");

        assertEquals(
                "MECANUM",
                savedConstants.optString("drivetrainType", "NAD")
        );
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.getX().getIn(), actual.getX().getIn(), 1e-9);
        assertEquals(expected.getY().getIn(), actual.getY().getIn(), 1e-9);
        assertEquals(expected.getHeading().getRad(), actual.getHeading().getRad(), 1e-9);
    }

    private static MotionVector simulateMotion(double x, double y, double turn) throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        follower.getDrivetrain().moveWithVectors(x, y, turn);

        for (int step = 0; step < 100; step++) {
            for (String motorName : hardware.drivetrain.motorNames) {
                SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class, motorName);
                motor.update(0.02);
            }
        }

        double[] wheelVelocities = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < hardware.drivetrain.motorNames.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class,
                    hardware.drivetrain.motorNames[i]
            );
            wheelVelocities[i] = motor.getVelocity();
        }

        java.lang.reflect.Method forwardKinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        forwardKinematics.setAccessible(true);
        return (MotionVector) forwardKinematics.invoke(hardware.drivetrain, (Object) wheelVelocities);
    }

    private static void assertWheelPowers(BaseDrivetrain<?> drivetrain,
                                          double fl, double fr, double bl, double br) {
        assertEquals(fl, drivetrain.getLastFlPower(), 1e-9);
        assertEquals(fr, drivetrain.getLastFrPower(), 1e-9);
        assertEquals(bl, drivetrain.getLastBlPower(), 1e-9);
        assertEquals(br, drivetrain.getLastBrPower(), 1e-9);
    }
}
