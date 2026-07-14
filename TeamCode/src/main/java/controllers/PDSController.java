package controllers;

import geometry.Angle;

/**
 * A general purpose PD controller with static friction compensation specifically made for
 * controlling robot movement in a one-dimensional axis.
 *
 * <ul>
 * <li><b>kP</b> - proportional gain (how aggressively the controller responds to error)</li>
 * <li><b>kD</b> - derivative gain (how much the controller accounts for error rate of change)</li>
 * <li><b>kS</b> - minimum power (a constant power added in the error's direction to overcome
 * static forces)</li>
 * </ul>
 *
 * <p>
 * The controller uses a soft sign function to smooth the kS term, which helps prevent
 * overshooting and oscillation.
 * </p>
 *
 * <p>
 * Special thanks to Wolfpack Machina (18438) for inspiration for this controller
 * </p>
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author DrPixelCat - 7842 alum
 */
public class PDSController {
    // TODO: I checked these on Desmos and it looks good, but they might need to be changed
    public static final double LINEAR_SMOOTHING_CONSTANT = 0.7; // In
    public static final double ANGULAR_SMOOTHING_CONSTANT = 0.07; // Rad
    private double smoothingConstant = LINEAR_SMOOTHING_CONSTANT;

    private PDSCoefficients coeffs;

    private boolean angularController = false;

    protected double target = 0.0;
    protected double lastError = 0.0;
    protected boolean timeAnomaly = false;
    private boolean firstRun = true;
    private long lastTimestamp;

    /**
     * A simple class to hold the PDSCoefficients for the PDSController.
     */
    public static class PDSCoefficients {
        public double kP, kD, kS;

        /**
         * Creates a PDSCoefficients object with the given values. Check the
         * {@link PDSController controller} documentation for what each coefficient does.
         */
        public PDSCoefficients(double kP, double kD, double kS) {
            this.kP = kP;
            this.kD = kD;
            this.kS = kS;
        }

        /**
         * Creates a PDSCoefficients object with all coefficients set to 0.
         */
        public PDSCoefficients() { this(0.0, 0.0, 0.0); }

        public void setkP(double kP) { this.kP = kP; }
        public void setkD(double kD) { this.kD = kD; }
        public void setkS(double kS) { this.kS = kS; }
    }

    /** @param coefficients the {@link PDSCoefficients} to use for the controller */
    public PDSController(PDSCoefficients coefficients) {
        this.setCoefficients(coefficients);
        this.lastTimestamp = System.nanoTime();
    }

    /** @param PDSCoefficients the {@link PDSCoefficients} to use for the controller */
    public void setCoefficients(PDSCoefficients PDSCoefficients) { this.coeffs = PDSCoefficients; }

    /** @return the current {@link PDSCoefficients} being used by the controller */
    public PDSCoefficients getCoefficients() { return this.coeffs; }

    /**
     * Sets the controller to be an angular controller. This should be called if the controller is
     * being used to control an angular value
     */
    public void setAngularController() {
        this.angularController = true;
        this.smoothingConstant = ANGULAR_SMOOTHING_CONSTANT;
    }

    /**
     * Resets the controller state. Call this right before starting a new movement to prevent
     * derivative kick and reset the timer.
     */
    public void reset() {
        this.firstRun = true;
        this.lastTimestamp = System.nanoTime();
    }

    /**
     * Calculates the output directly from a pre-calculated error.
     *
     * @param error The calculated error (Target - Current)
     * @return The control output
     */
    public synchronized double calculate(double error) {
        long currentNano = System.nanoTime();

        // Nano seconds to seconds
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        // Detect if loop is too fast (div by zero risk) or too slow (integral/derivative spike)
        timeAnomaly = deltaTime < 1E-6 || deltaTime > 0.15;

        double actualError = angularController ? Angle.normalize(error) : error; // 0 to 2pi

        if (firstRun) {
            lastError = actualError; // Prevents derivative kick from 0
            timeAnomaly = true;
            firstRun = false;
        }

        double p = this.coeffs.kP * error;
        double d = this.coeffs.kD * (timeAnomaly ? 0.0 : (error - lastError) / deltaTime);
        double s = this.coeffs.kS * (error / (Math.abs(error) + smoothingConstant));

        lastTimestamp = currentNano;
        lastError = actualError;

        return p + d + s;
    }
}