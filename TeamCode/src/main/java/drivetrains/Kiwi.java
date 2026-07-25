package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Kiwi (also known as Killough or Three-Wheel Omni) drivetrain controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Kiwi extends BaseDrivetrain<Kiwi.Constants> {
    private final double sqrt3over2 = Math.sqrt(3) / 2;

    public Kiwi(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.KIWI);
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Kiwi kinematics explanation: https://www.youtube.com/watch?v=n6TWzzj74gk&t=27
        setPowers(
                y / 2 - x * sqrt3over2 - turn,
                y / 2 + x * sqrt3over2 - turn,
                -y - turn, // Back motor = back left motor in the drivetrain configuration
                0 // Back right motor isn't used
        );
    }

    /** Configuration class for Kiwi/Killough/Thee-Wheel Omni drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        @Override
        public Kiwi build(HardwareMap hardwareMap) {
            if (flMotorConfig == null || frMotorConfig == null || blMotorConfig == null) {
                throw new IllegalArgumentException(
                        "All 3 motor configs must be provided for a kiwi drivetrain."
                );
            }

            return new Kiwi(this, hardwareMap);
        }

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

        /** Sets the back motor configuration. */
        public Constants setBackMotor(Motor Motor) {
            this.blMotorConfig = Motor; // Back uses the back left motor object
            return this;
        }
    }
}