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
    //Todo add vector class
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
    public double maxPower = 1.0;

    //booleans
    public boolean useBrakingMode = false;
    public boolean useFeedForward = true;




    //default constructor
    public MecanumConstants(){

    }

    //getter methods

    /**
     * getter for the motor names
     * @return String [4] of the motor names as shown below
     */
    public String [] getMotorNames(){
        String [] toReturn = new String[4];
        toReturn[0] = leftFrontMotorName;
        toReturn[1] = leftRearMotorName;
        toReturn[2] = rightFrontMotorName;
        toReturn[3] = rightRearMotorName;
        return  toReturn;
    }
    /**
     * getter for the motor directions
     * @return DcMotorSimple.Direction [4] of the motor names as shown below
     */
    public DcMotorSimple.Direction [] getMotorDirections(){
        DcMotorSimple.Direction [] toReturn = new DcMotorSimple.Direction[4];
        toReturn[0] = leftFrontDirection;
        toReturn[1] = leftRearDirection;
        toReturn[2] = rightFrontDirection;
        toReturn[3] = rightRearDirection;
        return  toReturn;
    }



}

