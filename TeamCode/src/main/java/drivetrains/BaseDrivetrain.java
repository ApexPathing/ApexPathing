package drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

/**
 * Base class for all drivetrain controllers.
 *
 * <p>
 * This class handles motor initialization and provides common methods for driving and setting motor
 * powers. Specific drivetrain types (like Tank, Mecanum, etc.) should extend this class and
 * implement the moveWithVectors method to define how the drive, strafe, and turn vectors are
 * translated into motor powers.
 * </p>
 *
 * @param <T> the type of drivetrain configuration this drivetrain uses, which must extend
 *        {@link BaseDrivetrainConstants}
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrain<T extends BaseDrivetrainConstants<T>> {
    protected T constants;

    public enum DrivetrainType {
        COAXIAL_SWERVE,
        DUAL_ACTUATED,
        KIWI,
        MECANUM,
        TANK
    }

    private final DrivetrainType drivetrainType;
    private final boolean isHolonomic;

    // Note: front motors are guaranteed to be non-null, but rear motors may be null if not needed
    protected DcMotorEx flMotor, frMotor, blMotor, brMotor;

    // Power change deadzone to prevent unnecessary motor updates
    private static final double POWER_TOLERANCE = 0.005;
    private double lastFlPower, lastFrPower, lastBlPower, lastBrPower = 0.0;

    /**
     * Your drivetrain class constructor should call this super constructor to initialize motors and
     * store the configuration.
     *
     * @param constants your drivetrain configuration object that is a child of {@link BaseDrivetrainConstants}
     * @param hardwareMap the hardware map to use for initializing motors
     */
    public BaseDrivetrain(T constants, HardwareMap hardwareMap, DrivetrainType drivetrainType) {
        if (Objects.equals(constants.flMotorConfig.getName(), "defaultMotorName")) {
            throw new IllegalArgumentException("Front left motor configuration is not set in the drivetrain constants.");
        }
        if (Objects.equals(constants.frMotorConfig.getName(), "defaultMotorName")) {
            throw new IllegalArgumentException("Front right motor configuration is not set in the drivetrain constants.");
        }
        flMotor = constants.flMotorConfig.build(hardwareMap);
        frMotor = constants.frMotorConfig.build(hardwareMap);

        if (constants.blMotorConfig != null) {
            if (Objects.equals(constants.blMotorConfig.getName(), "defaultMotorName")) {
                throw new IllegalArgumentException("Back left motor configuration is not set in the drivetrain constants.");
            }
            blMotor = constants.blMotorConfig.build(hardwareMap);
        }
        if (constants.brMotorConfig != null) {
            if (Objects.equals(constants.brMotorConfig.getName(), "defaultMotorName")) {
                throw new IllegalArgumentException("Back right motor configuration is not set in the drivetrain constants.");
            }
            brMotor = constants.brMotorConfig.build(hardwareMap);
        }

        this.constants = constants;
        this.drivetrainType = drivetrainType;
        this.isHolonomic = drivetrainType != DrivetrainType.TANK;
    }

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors.
     * The values are normalized and applied to the motors according to the mecanum drive formulas.
     *
     * @param x the forward/backward movement vector (positive for forward, negative for
     *          backward)
     * @param y the left/right movement vector (positive for left, negative for right)
     * @param turn the rotation vector (positive for counterclockwise, negative for clockwise)
     */
    public abstract void moveWithVectors(double x, double y, double turn);

    /**
     * Drives the robot with provided joystick inputs and the robot's current heading. This method
     * is meant for field-centric control. If you are using robot-centric control, the robotHeading
     * parameter will be ignored, you can use the other drive method that doesn't require the
     * robot's heading.
     *
     * @param x forward/backward joystick input (positive for forward, negative for backward)
     * @param y left/right joystick input (positive for left, negative for right)
     * @param turn rotation joystick input (positive for counterclockwise, negative for clockwise)
     * @param robotHeadingRad current heading of the robot in radians, only used for field centric
     */
    public void drive(double x, double y, double turn, double robotHeadingRad) {
        double adjX, adjY;
        if (!constants.robotCentric) { // Field centric
            double cos = Math.cos(-robotHeadingRad);
            double sin = Math.sin(-robotHeadingRad);
            adjX = x * cos - y * sin;
            adjY = x * sin + y * cos;
        } else {
            adjX = x;
            adjY = y;
        }
        moveWithVectors(adjX, adjY, turn);
    }

    /**
     * Drives the robot with provided joystick inputs. This method is meant for robotic-centric
     * control. If you are using field-centric control, you have to use the other drive method that
     * requires the robot's current heading to be passed in as a parameter.
     *
     * @param x forward/backward joystick input (positive for forward, negative for backward)
     * @param y left/right joystick input (positive for left, negative for right)
     * @param turn rotation joystick input (positive for counterclockwise, negative for clockwise)
     */
    public void drive(double x, double y, double turn) { drive(x, y, turn, 0); }

    /** @return the drivetrain type of this drivetrain */
    public DrivetrainType getDrivetrainType() {
        return drivetrainType;
    }

    /** @return Whether the drivetrain is currently in a holonomic state or not */
    public boolean isHolonomic() {
        return isHolonomic;
    }

    /**
     * Sets the power for each drivetrain motor, applying limits from the configurations. If your
     * drivetrain class does not use all 4 motors, just pass 0 for the motors you don't use.
     */
    public void setPowers(double flPower, double frPower, double blPower, double brPower) {
        // Motor power limiting
        double max = Math.max(0, Math.abs(flPower));
        max = Math.max(max, Math.abs(frPower));
        if (blMotor != null) max = Math.max(max, Math.abs(blPower));
        if (brMotor != null) max = Math.max(max, Math.abs(brPower));
        if (max > constants.maxPower) {
            flPower = (flPower / max) * constants.maxPower;
            frPower = (frPower / max) * constants.maxPower;
            if (blMotor != null) blPower = (blPower / max) * constants.maxPower;
            if (brMotor != null) brPower = (brPower / max) * constants.maxPower;
        }

        // Write to motors only if the change exceeds the tolerance
        if (Math.abs(flPower - lastFlPower) > POWER_TOLERANCE) {
            flMotor.setPower(flPower);
            lastFlPower = flPower;
        }
        if (Math.abs(frPower - lastFrPower) > POWER_TOLERANCE) {
            frMotor.setPower(frPower);
            lastFrPower = frPower;
        }
        if (blMotor != null && Math.abs(blPower - lastBlPower) > POWER_TOLERANCE) {
            blMotor.setPower(blPower);
            lastBlPower = blPower;
        }
        if (brMotor != null && Math.abs(brPower - lastBrPower) > POWER_TOLERANCE) {
            brMotor.setPower(brPower);
            lastBrPower = brPower;
        }
    }

    /**
     * Stop all drivetrain actuators
     */
    public void stop() { setPowers(0, 0, 0, 0); }
}
