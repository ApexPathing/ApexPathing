package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Tank drivetrain controller (supports 2 and 4 motor configurations)
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Tank extends BaseDrivetrain<Tank.Constants> {
    public Tank(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.TANK);
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Tank kinematics explanation (FRC, more in depth): https://www.youtube.com/watch?v=Ym34WI2rSdc
        // For FTC, not in depth, but shows simple code: https://www.youtube.com/watch?v=pREkiGl9yi0
        // 2 motor tank uses the front motors only, 4 motor tank uses all motors with the same power
        setPowers(x - turn, x + turn, x - turn, x + turn);
    }

    /** Configuration class for Tank drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        @Override
        public Tank build(HardwareMap hardwareMap) { return new Tank(this, hardwareMap); }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(Motor Motor) {
            this.flMotorConfig = Motor;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(Motor Motor) {
            this.frMotorConfig = Motor;
            return this;
        }

        /** Sets the back left motor configuration. Do not use this for 2 wheel tank */
        public Constants setBackLeftMotor(Motor Motor) {
            this.blMotorConfig = Motor;
            return this;
        }

        /** Sets the back right motor configuration. Do not use this for 2 wheel tank */
        public Constants setBackRightMotor(Motor Motor) {
            this.brMotorConfig = Motor;
            return this;
        }
    }
}