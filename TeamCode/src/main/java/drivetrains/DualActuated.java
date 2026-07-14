package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

/**
 * Drivetrain class that physically actuates between holonomic and high-traction kinematics.
 * Supports Butterfly, locking mecanum, and similar drivetrains.
 *
 * @author DrPixelCat - 7842 alum
 */
public class DualActuated extends BaseDrivetrain<DualActuated.Constants> {
    public enum DriveState {
        TANK,  // Locked rollers or deployed traction wheels (Tank kinematics)
        HOLONOMIC  // Unlocked rollers or retracted traction wheels (Mecanum kinematics)
    }

    private DriveState state;
    private final List<Actuator> actuators = new ArrayList<>();

    public DualActuated(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.DUAL_ACTUATED);

        if (this.constants.blMotorConfig == null || this.constants.brMotorConfig == null) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for an Actuated " +
                            "Dual Drivetrain."
            );
        }

        for (Actuator actuator : this.constants.actuators) {
            actuator.init(hardwareMap);
            actuators.add(actuator);
        }

        // Apply the initial state defined in the configuration
        applyState(this.constants.initialState);
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        if (state == DriveState.TANK) {
            setPowers(x - turn, x + turn,
                    x - turn, x + turn);
        } else {
            setPowers(x - y - turn, x + y + turn,
                    x + y - turn, x - y + turn
            );
        }
    }

    @Override
    public boolean isHolonomic() {
        return state != DriveState.TANK;
    }

    /** Sets the configuration to the TRACTION state */
    public void activateTractionState() {
        if (this.state != DriveState.TANK) {
            applyState(DriveState.TANK);
        }
    }

    /** Sets the configuration to the HOLONOMIC state */
    public void activateHolonomicState() {
        if (this.state != DriveState.HOLONOMIC) {
            applyState(DriveState.HOLONOMIC);
        }
    }

    /** @return the current state of the drivetrain (TANK or HOLONOMIC) */
    public DriveState getDriveState() {
        return state;
    }

    private void applyState(DriveState newState) {
        this.state = newState;
        for (Actuator actuator : actuators) {
            actuator.servo.setPosition(state == DriveState.TANK ? actuator.tankPos :
                    actuator.holonomicPos);
        }
    }

    /** Helper class to create servo and store states */
    public static class Actuator {
        Servo servo;
        final String name;
        final double tankPos;
        final double holonomicPos;

        Actuator(String name, double tankPos, double holonomicPos) {
            this.name = name;
            this.tankPos = tankPos;
            this.holonomicPos = holonomicPos;
        }

        public void init(HardwareMap hardwareMap) {
            this.servo = hardwareMap.get(Servo.class, name);
        }
    }

    /**
     * Configuration class for an Actuated Dual Drivetrain.
     */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        // Define the initial startup state (defaults to Holonomic)
        public DriveState initialState = DriveState.HOLONOMIC;
        public final List<Actuator> actuators = new ArrayList<>();

        @Override
        public DualActuated build(HardwareMap hardwareMap) {
            return new DualActuated(this, hardwareMap);
        }

        /** Sets the initial startup state of the drivetrain */
        public Constants setInitialState(DriveState initialState) {
            this.initialState = initialState;
            return this;
        }

        public Constants setFrontLeftMotor(Motor Motor) {
            this.flMotorConfig = Motor;
            return this;
        }

        public Constants setFrontRightMotor(Motor Motor) {
            this.frMotorConfig = Motor;
            return this;
        }

        public Constants setBackLeftMotor(Motor Motor) {
            this.blMotorConfig = Motor;
            return this;
        }

        public Constants setBackRightMotor(Motor Motor) {
            this.brMotorConfig = Motor;
            return this;
        }

        /**
         * Adds an actuation servo to the drivetrain configuration.
         * * @param name The hardware map name of the servo
         *
         * @param tractionPosition  The physical servo position for the high-traction state (e.g.
         *                          locked or wheel deployed)
         * @param holonomicPosition The physical servo position for the holonomic state (e.g.
         *                          unlocked or wheel retracted)
         */
        public Constants addActuator(String name, double tractionPosition,
                                     double holonomicPosition) {
            actuators.add(new Actuator(name, tractionPosition, holonomicPosition));
            return this;
        }
    }
}