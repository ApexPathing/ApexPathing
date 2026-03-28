package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

<<<<<<< HEAD
<<<<<<< HEAD
import org.firstinspires.ftc.robotcore.external.Telemetry;

=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
import org.firstinspires.ftc.robotcore.external.Telemetry;


>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
import java.util.List;

/**
 * Mecanum Drivetrain controller class
 * @author Xander Haemel - 31616 - 404 Not Found
 * @author Sohum Arora - 22985 Paraducks
 */
public class MecanumDrive extends Drivetrain{
    MecanumConstants mechconstants;
    private final double maxPower = 1.0;
    private final List<DcMotorEx> motors;
    private final double[] lastMotorPowers;

    //motors
    DcMotorEx lf;
    DcMotorEx lr;
    DcMotorEx rf;
    DcMotorEx rr;

<<<<<<< HEAD
<<<<<<< HEAD
    boolean useBrakeMode;

=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
    boolean useBrakeMode;


>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    //Constructor
    public MecanumDrive(HardwareMap hardwareMap, MecanumConstants constants, List<DcMotorEx> motors, double[] lastMotorPowers){
        this.mechconstants = constants;
        this.motors = motors;
        this.lastMotorPowers = lastMotorPowers;
        String [] names =  mechconstants.getMotorNames();
        lf = hardwareMap.get(DcMotorEx.class, names[0]);
        lr = hardwareMap.get(DcMotorEx.class, names[1]);
        rf = hardwareMap.get(DcMotorEx.class, names[2]);
        rr = hardwareMap.get(DcMotorEx.class, names[3]);

        //Reversing motors
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * uses the mecanum equation to set powers of the wheels based on inputs
     * @param drive the power to drive forward
     * @param strafe the power to strafe
     * @param turn the power to turn
     */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af

    /**
     * sets the calculated powers of each motor
     * @param drive power to drive forward
     * @param strafe power to strafe
     * @param turn power to turn
     */
<<<<<<< HEAD
=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======

>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    public void setPowers(double drive, double strafe, double turn){
        double [] powers = calculatePower(drive, strafe, turn);
        lf.setPower(powers[0]);
        lr.setPower(powers[1]);
        rf.setPower(powers[2]);
        rr.setPower(powers[3]);
    }

    /**
     * caluclates the powers
     * @param drive the power to drive forward
     * @param strafe the power to strafe
     * @param turn the power to turn
     */
    private double [] calculatePower(double drive, double strafe, double turn){
        double lfPower = drive + strafe + turn;
        double lrPower = drive - strafe + turn;
        double rfPower = drive -strafe - turn;
        double rrPower = drive + strafe - turn;
        //format powers
        double [] calculatedPowers = {lfPower,lrPower,rfPower,rrPower};
        //return
        return  calculatedPowers;
    }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af

    /**
     * robot centric drive
     * @param x
     * @param y
     * @param turn
     */
<<<<<<< HEAD
=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    public void botCentricDrive(double x, double y, double turn) {
        double adjX = deadzone(x, 0.05);
        double adjY = deadzone(y, 0.05);
        double adjTurn = deadzone(turn, 0.05);

        double[] powers = {
                adjY + adjX + adjTurn,   // left front
                adjY - adjX + adjTurn,   // left rear
                adjY - adjX - adjTurn,   // right front
                adjY + adjX - adjTurn    // right rear
        };

        normalizePowers(powers);
        runDrive(powers);
    }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af

    /**
     * field centric drive
     * @param x
     * @param y
     * @param turn
     * @param robotHeading
     */
    public void fieldCentricDrive(double x, double y, double turn, double robotHeading) {
        double cos    = Math.cos(-robotHeading);
        double sin    = Math.sin(-robotHeading);
        double fieldX = deadzone(x * cos - y * sin, 0.05);
        double fieldY = deadzone(x * sin + y * cos, 0.05);
        double adjTurn = deadzone(turn, 0.05);

        double[] powers = {
                fieldY + fieldX + adjTurn,  // left front
                fieldY - fieldX + adjTurn,  // left rear
                fieldY - fieldX - adjTurn,  // right front
                fieldY + fieldX - adjTurn   // right rear
        };

        normalizePowers(powers);
        runDrive(powers);
    }
<<<<<<< HEAD
=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    private static double deadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }
    private void normalizePowers(double[] powers) {
        double max = 0;
        for (double p : powers) {
            max = Math.max(max, Math.abs(p));
        }
        if (max > maxPower)
            for (int i = 0; i < powers.length; i++) powers[i] = (powers[i] / max) * maxPower;
    }
    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            boolean changed  = Math.abs(lastMotorPowers[i] - drivePowers[i]) > 0.01;
            boolean zeroed   = drivePowers[i] == 0 && lastMotorPowers[i] != 0;
            if (changed || zeroed) {
                lastMotorPowers[i] = drivePowers[i];
                setPower(motors.get(i), drivePowers[i]);
            }
        }
    }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    public void breakFollowing() {
        for (int i = 0; i < motors.size(); i++) lastMotorPowers[i] = 0;
        setPower(0);
        if (useBrakeMode) setMotorsToBrake();
        else setMotorsToFloat();
    }
    private void setMotorsToBrake() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorsToFloat() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Syncs live constants (useful with FTC Dashboard). */
    //abstract overrides
<<<<<<< HEAD
=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    @Override
    public void setPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }

    @Override
    public void setPower(List<DcMotorEx> motors, double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af

    @Override
    public void setPower(double power) {
        lf.setPower(power);
        lr.setPower(power);
        rf.setPower(power);
        rr.setPower(power);
    }

    @Override
    public void initDrive(HardwareMap hardwareMap, String lfName, String rfName, String lrName, String rrName) {
        lf = hardwareMap.get(DcMotorEx.class, lfName);
        rf = hardwareMap.get(DcMotorEx.class, rfName);
        lr = hardwareMap.get(DcMotorEx.class, lrName);
        rr = hardwareMap.get(DcMotorEx.class, rrName);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lf.setZeroPowerBehavior(behavior);
        lr.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rr.setZeroPowerBehavior(behavior);
    }

    //debuggig/tuning
    public void logMotors(Telemetry telemetry) {
        telemetry.addLine("---Power---");

        telemetry.addData("leftFront Power", lf.getPower());
        telemetry.addData("rightFront Power", rf.getPower());
        telemetry.addData("leftRear Power", lr.getPower());
        telemetry.addData("rightRear Power", rr.getPower());

        telemetry.addLine("---Velocity---");

        telemetry.addData("leftFront velocity", lf.getVelocity());
        telemetry.addData("rightFront velocity", rf.getVelocity());
        telemetry.addData("leftRear velocity", lr.getVelocity());
        telemetry.addData("rightRear velocity", rr.getVelocity());
    }
<<<<<<< HEAD
=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======

>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
}
