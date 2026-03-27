package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum COnstants class
 * @author Xander Haemel - 31616 404 Not Found
 */
public class MecanumConstants {
    //velocities (get from tuning)
    public final double xVelocity = 60; //inches per second
    public final double yVelocity = 60; //inches per second

    //dc motor names
    public final String leftFrontMotorName = "leftFront";
    public final String leftRearMotorName = "leftRear";
    public final String rightFrontMotorName = "rightFront";
    public final String rightRearMotorName = "rightRear";

    //motor directions
    public final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public final DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
    public final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public final DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD;

    //misc
    public double MaxPower = 1.0;

    //booleans
    public boolean useBrakingMode = false;
    public boolean useFeedForward = true;




    //default constructor
    public MecanumConstants(){

    }

}

