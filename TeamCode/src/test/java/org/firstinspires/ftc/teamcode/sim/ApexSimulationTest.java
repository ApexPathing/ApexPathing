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
import org.firstinspires.ftc.teamcode.apexpathing.ExampleAutoPath;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.firstinspires.ftc.teamcode.apexpathing.TeleOpTest;
import org.json.JSONObject;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

import core.Follower;
import core.FollowerConstants;
import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrain;
import geometry.Pose;
import geometry.GeometryFactory;
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

    @Test
    public void exampleAutoPathAndTurnConvergeInSimulation() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        configureKnownFollowerConstants();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        ExampleAutoPath auto = new ExampleAutoPath(follower, GeometryFactory.PoseMirror.NONE);

        follower.follow(auto.testPath);
        follower.update();
        assertEquals("Callback not triggered yet", auto.callbackMessage);
        double outboundCrossTrack = runMovement(hardware, follower, 12.0);
        feedforward.MotionParameters first = auto.testPath.getFeedforwardLut().getFFParams(0.0);
        assertTrue("Example auto path did not finish: pose=" + follower.getPose() +
                ", t=" + follower.getBestT() +
                ", crossTrack=" + follower.getCrossTrackErrorIn() +
                ", velocity=" + follower.getVelocity() +
                ", firstV=" + first.getTangentialVel() +
                ", firstA=" + first.getTangentialAccel() +
                ", powers=" + follower.getDrivetrain().getLastFlPower() + "," +
                follower.getDrivetrain().getLastFrPower() + "," +
                follower.getDrivetrain().getLastBlPower() + "," +
                follower.getDrivetrain().getLastBrPower(), !follower.isBusy());
        assertTrue("Example auto path stopped too far from its endpoint",
                follower.getPose().distanceTo(auto.testPath.getEndPose()).getIn() < 2.0);
        assertTrue("Outbound distance callback did not fire", auto.outboundCallbackTriggered);
        assertTrue("Outbound path cross-track error was excessive: " + outboundCrossTrack,
                outboundCrossTrack < 6.0);

        follower.follow(auto.testTurn);
        runMovement(hardware, follower, 8.0);
        assertTrue("Example auto turn did not finish", !follower.isBusy());
        assertTrue("Example auto turn stopped at the wrong heading",
                Math.abs(follower.getPose().getHeading().getShortestAngleTo(
                        auto.testTurn.getEndPose().getHeading()).getRad()) < Math.toRadians(3.0));
        assertTrue("Angular callback did not fire", auto.turnCallbackTriggered);

        follower.follow(auto.returnPath);
        double returnCrossTrack = runMovement(hardware, follower, 12.0);
        assertTrue("Reverse return path did not finish", !follower.isBusy());
        assertTrue("Reverse return path stopped too far from home: " + follower.getPose(),
                follower.getPose().distanceTo(auto.returnPath.getEndPose()).getIn() < 1.0);
        assertTrue("Reverse return path ended at the wrong heading",
                Math.abs(follower.getPose().getHeading().getShortestAngleTo(
                        auto.returnPath.getEndPose().getHeading()).getRad()) < Math.toRadians(3.0));
        assertTrue("Return distance callback did not fire", auto.returnCallbackTriggered);
        assertTrue("Reverse path cross-track error was excessive: " + returnCrossTrack,
                returnCrossTrack < 6.0);

        follower.follow(auto.strafeOutPath);
        double strafeOutCrossTrack = runMovement(hardware, follower, 8.0);
        assertTrue("Outbound strafe did not finish: pose=" + follower.getPose() +
                ", t=" + follower.getBestT() + ", velocity=" + follower.getVelocity(),
                !follower.isBusy());
        assertTrue("Outbound strafe stopped too far from its endpoint",
                follower.getPose().distanceTo(auto.strafeOutPath.getEndPose()).getIn() < 1.0);
        assertTrue("Outbound strafe cross-track error was excessive: " + strafeOutCrossTrack,
                strafeOutCrossTrack < 6.0);

        follower.follow(auto.strafeBackPath);
        double strafeBackCrossTrack = runMovement(hardware, follower, 8.0);
        assertTrue("Return strafe did not finish: pose=" + follower.getPose() +
                        ", t=" + follower.getBestT() + ", velocity=" +
                        follower.getVelocity(),
                !follower.isBusy());
        assertTrue("Full auto sequence did not return to the origin: " + follower.getPose(),
                follower.getPose().distanceTo(Pose.zero()).getIn() < 1.0);
        assertTrue("Full auto sequence did not finish at zero heading",
                Math.abs(follower.getPose().getHeading().getShortestAngleTo(
                        Pose.zero().getHeading()).getRad()) < Math.toRadians(3.0));
        assertTrue("Return strafe cross-track error was excessive: " + strafeBackCrossTrack,
                strafeBackCrossTrack < 6.0);
    }

    @Test
    public void velocityFeedbackTranslationCandidatesCompleteBothDirections() throws Exception {
        double center = 0.025076489028092973;
        double[] firstRoundGains = { center * 0.5, center, center * 1.5 };
        for (double gain : firstRoundGains) {
            runVelocityFeedbackTranslationTrial(gain);
        }
    }

    private static void runVelocityFeedbackTranslationTrial(double gain) throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        configureKnownFollowerConstants();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        follower.setVelocityFeedback(gain, 0.34);

        GeometryFactory factory = new GeometryFactory(follower)
                .setDistUnit(geometry.DistUnit.IN)
                .setAngleUnit(geometry.AngleUnit.DEG);
        Pose start = factory.pose(-24, 0, 0);
        Pose end = factory.pose(24, 0, 0);
        paths.movements.Path outbound = factory.path(start, end)
                .interpolateWith(paths.heading.InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();
        paths.movements.Path returning = factory.path(end, start)
                .interpolateWith(paths.heading.InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();

        follower.setPose(start);
        follower.follow(outbound);
        runMovement(hardware, follower, 10.0);
        assertTrue("Velocity-feedback outbound path did not finish at gain " + gain + ": " +
                        follower.getPose(),
                !follower.isBusy());

        follower.follow(returning);
        runMovement(hardware, follower, 10.0);
        assertTrue("Velocity-feedback return path did not finish at gain " + gain + ": " +
                        follower.getPose(),
                !follower.isBusy());
        assertTrue("Velocity-feedback return path did not settle at its endpoint: " +
                        follower.getPose(),
                follower.getPose().distanceTo(start).getIn() < 1.0);
    }

    @Test
    public void velocityFeedbackAngularCandidatesCompleteBothDirections() throws Exception {
        double center = 0.2466970500838876;
        double[] firstRoundGains = { center * 0.5, center, center * 1.5 };
        for (double gain : firstRoundGains) {
            runVelocityFeedbackAngularTrial(gain);
        }
    }

    private static void runVelocityFeedbackAngularTrial(double gain) throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        configureKnownFollowerConstants();
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs = new PDSCoefficients(
                2.189285375109406, 0.2466970500838876, 0.24375);
        constants.angularKV = 0.13651719918241684;
        constants.angularKA = 0.05331768750185841;
        constants.angularVelLimitRad = 6.958830137809905;
        constants.angularAccelLimitRad = 17.81772699663479;
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        follower.setVelocityFeedback(0.0251, gain);

        GeometryFactory factory = new GeometryFactory(follower)
                .setDistUnit(geometry.DistUnit.IN)
                .setAngleUnit(geometry.AngleUnit.DEG);
        Pose start = factory.pose(-24, 0, 0);
        Pose turned = factory.pose(-24, 0, 90);
        paths.movements.Turn outbound = factory.turn(start)
                .turnTo(turned.getHeading()).profiledBuild();
        paths.movements.Turn returning = factory.turn(turned)
                .turnTo(start.getHeading()).profiledBuild();

        follower.setPose(start);
        follower.follow(outbound);
        runMovement(hardware, follower, 8.0);
        assertTrue("Velocity-feedback outbound turn did not finish at gain " + gain + ": " +
                        follower.getPose(),
                !follower.isBusy());

        follower.follow(returning);
        runMovement(hardware, follower, 8.0);
        assertTrue("Velocity-feedback return turn did not finish at gain " + gain + ": " +
                        follower.getPose(),
                !follower.isBusy());
        assertTrue("Velocity-feedback return turn did not settle at zero heading",
                Math.abs(follower.getPose().getHeading().getRad()) < Math.toRadians(2.0));
    }

    @Test
    public void centripetalQuickArcsCompleteBothDirections() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        configureKnownFollowerConstants();
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        follower.setCentripetal(0.00664954474299773);

        GeometryFactory factory = new GeometryFactory(follower)
                .setDistUnit(geometry.DistUnit.IN)
                .setAngleUnit(geometry.AngleUnit.DEG);
        Pose start = factory.pose(-32, -16, 0);
        Pose middle = factory.pose(32, -16, 0);
        Pose end = factory.pose(32, 16, 90);
        paths.movements.Path outbound = factory.path(start, middle, end)
                .interpolateWith(paths.heading.InterpolationStyle.TANGENT_FORWARD)
                .quickBuild();
        paths.movements.Path returning = factory.path(end, middle, start)
                .interpolateWith(paths.heading.InterpolationStyle.TANGENT_BACKWARD)
                .quickBuild();

        follower.setPose(start);
        follower.follow(outbound);
        runMovement(hardware, follower, 15.0);
        assertTrue("Centripetal outbound arc did not finish: pose=" + follower.getPose() +
                        ", t=" + follower.getBestT() + ", velocity=" +
                        follower.getVelocity(),
                !follower.isBusy());

        follower.follow(returning);
        runMovement(hardware, follower, 15.0);
        assertTrue("Centripetal return arc did not finish: pose=" + follower.getPose() +
                        ", t=" + follower.getBestT() + ", velocity=" +
                        follower.getVelocity(),
                !follower.isBusy());
        assertTrue("Centripetal return arc did not settle at its endpoint: " +
                        follower.getPose(),
                follower.getPose().distanceTo(start).getIn() < 1.0);
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.getX().getIn(), actual.getX().getIn(), 1e-9);
        assertEquals(expected.getY().getIn(), actual.getY().getIn(), 1e-9);
        assertEquals(expected.getHeading().getRad(), actual.getHeading().getRad(), 1e-9);
    }

    private static void configureKnownFollowerConstants() {
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs = new PDSCoefficients(3.32, 0.34, 0.24375);
        constants.translationalCoeffs = new PDSCoefficients(0.149, 0.024, 0.24375);
        constants.translationalKV = 0.01466;
        constants.translationalKA = 0.00716;
        constants.angularKV = 0.1364;
        constants.angularKA = 0.0648;
        constants.velocityFeedbackGain = 0.0;
        constants.angularVelocityFeedbackGain = 0.0;
        constants.kCentripetal = 0.0072;
        constants.forwardVelLimitIn = 64.8;
        constants.forwardAccelLimitIn = 132.6;
        constants.strafeVelLimitIn = 53.6;
        constants.strafeAccelLimitIn = 94.8;
        constants.angularVelLimitRad = 6.96;
        constants.angularAccelLimitRad = 14.66;
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

    private static double runMovement(ApexSimulation.Hardware hardware, Follower follower,
                                      double timeoutSeconds) throws Exception {
        long deadline = System.nanoTime() + (long) (timeoutSeconds * 1e9);
        double maximumCrossTrackError = 0.0;
        while (follower.isBusy() && System.nanoTime() < deadline) {
            stepPhysics(hardware, 0.02);
            follower.update();
            maximumCrossTrackError = Math.max(maximumCrossTrackError,
                    Math.abs(follower.getCrossTrackErrorIn()));
            Thread.sleep(20);
        }
        return maximumCrossTrackError;
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

    private static void assertWheelPowers(BaseDrivetrain<?> drivetrain,
                                          double fl, double fr, double bl, double br) {
        assertEquals(fl, drivetrain.getLastFlPower(), 1e-9);
        assertEquals(fr, drivetrain.getLastFrPower(), 1e-9);
        assertEquals(bl, drivetrain.getLastBlPower(), 1e-9);
        assertEquals(br, drivetrain.getLastBrPower(), 1e-9);
    }
}
