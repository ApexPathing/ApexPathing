package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedMecanum;
import org.codeblooded.ftcodesim.input.DefaultKeybinds;
import org.codeblooded.ftcodesim.physics.MotionVector;
import org.codeblooded.ftcodesim.physics.RobotGeometry;
import org.codeblooded.ftcodesim.simulator.FTCodeSim;
import org.codeblooded.ftcodesim.simulator.FTCodeSimTelemetryInstaller;
import org.codeblooded.ftcodesim.ascope.ApexAdvantageScopeLayout;
import org.codeblooded.ftcodesim.simulator.SimConfig;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;

import java.io.File;
import java.io.IOException;

import core.ApexStorage;

/** Shared FTCodeSim configuration used by every Apex Pathing OpMode. */
public final class ApexSimulation {
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String PINPOINT = "pinpoint";

    // FTCodeSim's DECODE field is represented in corner-origin coordinates.
    static final double FIELD_CENTER_INCHES = 141.5 / 2.0;

    private ApexSimulation() { }

    public static SimConfig createConfig() {
        SimConfig config = new SimConfig();
        config.gamepad1Keybinds = new DefaultKeybinds();
        config.gamepad2Keybinds = new DefaultKeybinds();
        config.simHardwareMap = createHardware().hardwareMap;
        config.loopTimeMs = 20;
        return config;
    }

    public static FTCodeSim createSimulator() throws IOException {
        FTCodeSim simulator = new FTCodeSim(createConfig());
        ApexAdvantageScopeLayout.install();
        installTelemetryAdapter(simulator);
        return simulator;
    }

    static Hardware createHardware() {
        configureDesktopStorage();

        SimMecanumConfig config = new SimMecanumConfig();
        // The FTC SDK applies the configured right-side motor reversals before physical wheel
        // motion. FTCodeSim's SimMotor ignores setDirection(), and its mecanum model consumes the
        // four logical powers directly with the opposite lateral/turn convention. Mirroring the
        // model's wheel slots preserves the production motor mix while making +Y and +heading
        // agree with Apex in the simulator.
        config.frontLeftMotorName = FRONT_RIGHT_MOTOR;
        config.frontRightMotorName = FRONT_LEFT_MOTOR;
        config.backLeftMotorName = BACK_RIGHT_MOTOR;
        config.backRightMotorName = BACK_LEFT_MOTOR;

        // These are the measured Code Blooded drivetrain values from FTCodeSim's DECODE example.
        config.wheelbase = 9.37008;
        config.trackWidth = 9.13386;
        config.wheelRadius = 1.889765;
        config.staticVelocityRegion = 2;
        config.staticFriction = 45;
        config.maxAcceleration = 150;
        config.maxVelocity = 75;
        config.naturalDeceleration = 40;
        config.strafeEfficiency = 0.80;
        config.robotGeometry = new RobotGeometry(12, 18, 2, 0);

        SimulatedDrivetrain drivetrain = new SimulatedMecanum(config);
        SimHardwareMap hardwareMap = new StableSimHardwareMap();
        hardwareMap.register(drivetrain);

        // Put Apex's centered origin at the center of FTCodeSim's corner-origin field.
        drivetrain.setPosition(new MotionVector(
                FIELD_CENTER_INCHES,
                FIELD_CENTER_INCHES,
                0.0
        ));
        hardwareMap.register(PINPOINT, new SimApexPinpoint(drivetrain, FIELD_CENTER_INCHES));
        return new Hardware(hardwareMap, drivetrain);
    }

    private static void configureDesktopStorage() {
        System.setProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, "true");
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
    }

    private static void installTelemetryAdapter(FTCodeSim simulator) {
        FTCodeSimTelemetryInstaller.install(simulator, new ApexSimTelemetry(simulator));
    }

    static final class Hardware {
        final SimHardwareMap hardwareMap;
        final SimulatedDrivetrain drivetrain;

        Hardware(SimHardwareMap hardwareMap, SimulatedDrivetrain drivetrain) {
            this.hardwareMap = hardwareMap;
            this.drivetrain = drivetrain;
        }
    }
}
