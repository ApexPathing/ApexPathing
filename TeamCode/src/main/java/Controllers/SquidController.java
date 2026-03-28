package Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A squID controller for follower
 * It takes the square root of the P, but otherwise it's pretty normal for a controller
 * @author Xander Haemel - 31616
 */
public class SquidController {
    public double kP, kI, kD;
    public double goal = 0;
    private double integralSum = 0;
    private double derivative;
    private double lastError = 0;
    private double error = 0;
    private double motorDeadzone = 0.05;

    private double lastTimestamp = 0;
    private final ElapsedTime timer;

    /**
     * default constuctor
     * @param kP is the kP of the controller
     * @param kI is the kI of the controller
     * @param kD is the kD of the controller
     */
    public SquidController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer = new ElapsedTime();
        timer.startTime();
    }

    public void setGoal(double newGoal) {
        this.goal = newGoal;
    }

    /**
     * Calculates the PIDF output.
     * @param currentPosition the current position of the mechanism
     * @return the power output
     */
    public synchronized double calculate(double currentPosition) {
        double timestamp = timer.milliseconds() / 1000;
        double deltaTime = timestamp - lastTimestamp;

        if (deltaTime == 0) return 0;

        //p term, but its special because its a square root
        error = goal - currentPosition;
        //absolute value in case the error is negative, math.signum ensures the robot goes the right way
        double kPOut = kP * Math.sqrt(Math.abs(error)) * Math.signum(error);

        //i term, standard PID calculations
        integralSum += error * deltaTime;
        double iLimit = 0.25;
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        double kIOut = kI * integralSum;

        derivative = (error - lastError) / deltaTime;
        double kDOut = kD * derivative;

        //reset vars
        lastTimestamp = timestamp;
        lastError = error;

        //power variable for ease of calculation
        final double power = kPOut + kIOut + kDOut;

        //deadband, turns the motors off when the power is too slow to move the robot
        if(Math.abs(power) < motorDeadzone){
            return 0;
        }

        //limit power to -1, and 1 for safety
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
