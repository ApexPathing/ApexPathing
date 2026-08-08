package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.simulator.OpModeRegister;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.firstinspires.ftc.teamcode.apexpathing.TeleOpTest;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

import core.Follower;
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
    public void pinpointAdapterConvertsFieldCoordinatesAndHeading() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimApexPinpoint pinpoint = (SimApexPinpoint) hardware.hardwareMap.get(
                Pinpoint.Driver.class,
                ApexSimulation.PINPOINT
        );

        hardware.drivetrain.setPosition(new MotionVector(
                ApexSimulation.FIELD_CENTER_INCHES + 12.0,
                ApexSimulation.FIELD_CENTER_INCHES - 5.0,
                -Math.PI / 3.0
        ));
        pinpoint.update(0.02);

        Pose pose = pinpoint.getPosition();
        assertEquals(12.0, pose.getX().getIn(), 1e-9);
        assertEquals(5.0, pose.getY().getIn(), 1e-9);
        assertEquals(Math.PI / 3.0, pose.getHeading().getRad(), 1e-9);
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.getX().getIn(), actual.getX().getIn(), 1e-9);
        assertEquals(expected.getY().getIn(), actual.getY().getIn(), 1e-9);
        assertEquals(expected.getHeading().getRad(), actual.getHeading().getRad(), 1e-9);
    }
}
