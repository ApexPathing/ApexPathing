package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import geometry.Angle;
import geometry.Dist;

/**
 * Abstract class implemented by all drivetrain configuration classes
 *
 * <p>
 * When creating a drivetrain configuration, you must extend this class and implement the build()
 * method to return an instance of the corresponding drivetrain class using your configuration
 * class.
 * Your constants should have a public scope and be initialized with default values.
 * </p>
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrainConstants<T extends BaseDrivetrainConstants<T>> {
    // Motors (only 2 motors are guaranteed, others are optional and may be null)
    // Child classes should handle setting these up as needed
    public Motor flMotorConfig = new Motor();
    public Motor frMotorConfig = new Motor();
    public Motor blMotorConfig = null;
    public Motor brMotorConfig = null;

    public double maxPower = 1.0;
    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);

    // Other constants
    public boolean robotCentric = true; // Robot or field centric control

    /**
     * Builds and returns an instance of the corresponding drivetrain class using this
     * configuration.
     */
    public abstract BaseDrivetrain<?> build(HardwareMap hardwareMap);

    /** Set the maximum motor output limit for the drivetrain. The default is 1.0. */
    @SuppressWarnings("unchecked")
    public T setMaxPower(double maxPower) {
        this.maxPower = Math.max(Math.min(0.0, maxPower), 1.0);
        return (T) this;
    }

    /**
     * Set whether the drivetrain should use robot-centric controls or field-centric controls in
     * TeleOp. The default is true (robot-centric).
     */
    @SuppressWarnings("unchecked")
    public T setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return (T) this;
    }

    /**
     * Set how close the drivetrain must be to the target heading to be considered "on target". The
     * default is 1 degree.
     */
    @SuppressWarnings("unchecked")
    public T setHeadingTolerance(Angle headingTolerance) {
        this.headingTolerance = headingTolerance;
        return (T) this;
    }

    /**
     * Set how close the drivetrain must be to the target position to be considered "on target". The
     * default is 0.5 inches.
     */
    @SuppressWarnings("unchecked")
    public T setDistanceTolerance(Dist distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return (T) this;
    }

    /**
     * Get the motor configuration for the front left motor.
     */
    public Motor getFlMotorConfig() { return flMotorConfig; }

    /**
     * Get the motor configuration for the front right motor.
     */
    public Motor getFrMotorConfig() { return frMotorConfig; }

    /**
     * Get the motor configuration for the back left motor. May be null if not set.
     */
    public Motor getBlMotorConfig() { return blMotorConfig; }

    /**
     * Get the motor configuration for the back right motor. May be null if not set.
     */
    public Motor getBrMotorConfig() { return brMotorConfig; }
}
