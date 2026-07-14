package drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A class used to easily create and configure {@link DcMotorEx} objects.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Atharv G. - 13085 Bionic Dutch
 */
public class Motor {
    private final String name;
    private boolean reversed = false;
    private boolean floatMode = false;
    private RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;

    /**
     * Creates a motor constants with the given name and defaults (not reversed, brake mode, run
     * without encoder).
     */
    public Motor(String name) { this.name = name; }

    /**
     * Creates a default motor constants ("defaultMotorName", not reversed, brake mode, run without
     * encoder).
     */
    public Motor() { this("defaultMotorName"); }

    /**
     * Sets the motor direction to reverse instead of teh default forward
     */
    public Motor reverse() {
        this.reversed = true;
        return this;
    }

    /**
     * Sets the motors zero power behavior to float instead of the default brake
     */
    public Motor floatMode() {
        this.floatMode = true;
        return this;
    }

    /**
     * @param mode The {@link RunMode} of the motor. Defaults to RUN_WITHOUT_ENCODER.
     */
    public Motor runWith(RunMode mode) {
        this.runMode = mode;
        return this;
    }

    /**
     * @param hardwareMap the {@link HardwareMap} to use for initializing the motor
     * @return a {@link DcMotorEx} from this MotorMetaData.
     */
    public DcMotorEx build(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? Direction.REVERSE : Direction.FORWARD);
        motor.setZeroPowerBehavior(floatMode ? ZeroPowerBehavior.FLOAT : ZeroPowerBehavior.BRAKE);
        motor.setMode(runMode);
        return motor;
    }

    public String getName() {return name;}

    public boolean getReversed() {return reversed;}

    public boolean getFloatMode() {return floatMode;}

    public RunMode getRunMode() {return runMode;}
}