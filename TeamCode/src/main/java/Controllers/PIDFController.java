package Controllers;

import com.apexpathing.util.Controller;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PIDF feedback controller.
 * @author Xander Haemel -31616
 */
public class PIDFController extends Controller {
    public double kP, kI, kD, kF;
    private double integralSum = 0;
    private double derivative;
    private double lastError = 0;
    private double error;
    private double lastTimestamp = 0;
    private final ElapsedTime timer;
    private double motorDeadzone = 0.05;

    //default constructor
    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer = new ElapsedTime();
        timer.startTime();
    }

    /**
     * Calculates the PIDF output.
     * @param currentPosition the current position of the mechanism
     * @return the power output
     */
    @Override
    public synchronized double calculate(double currentPosition) {
        double timestamp = timer.milliseconds() / 1000;
        double deltaTime = timestamp - lastTimestamp;

        if (deltaTime == 0) return 0;

        error = this.getGoal() - currentPosition;

        double kPOut = kP * error;

        integralSum += error * deltaTime;
        double iLimit = 0.25;
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        double kIOut = kI * integralSum;

        derivative = (error - lastError) / deltaTime;
        double kDOut = kD * derivative;

        double kFOut = kF * Math.signum(error);

        lastTimestamp = timestamp;
        lastError = error;

        double power = kPOut + kIOut + kDOut + kFOut;
        if(Math.abs(power) < motorDeadzone){
            power = 0;
        }
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
