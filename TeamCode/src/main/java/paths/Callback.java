package paths;

import com.qualcomm.robotcore.util.Range;

import geometry.Angle;

/**
 * A unified callback container capable of handling either distance-based progress triggers
 * or angular-based orientation triggers during robot movement execution.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Callback {
    public enum CallbackType {
        DISTANCE,
        ANGLE
    }

    private final CallbackType type;
    private final Runnable action;
    private boolean triggered = false;

    private final double s;
    private final Angle theta;

    /**
     * Constructs a physical path distance progress percentage callback.
     *
     * @param s The path completion percentage [0.0, 1.0].
     * @param action The code routine to execute when reached.
     */
    public Callback(double s, Runnable action) {
        this.type = CallbackType.DISTANCE;
        this.s = Range.clip(s, 0.0, 1.0);
        this.theta = null;
        this.action = action;
    }

    /**
     * Constructs an angular target sweep callback.
     *
     * @param theta The target field/robot orientation angle.
     * @param action The code routine to execute when reached.
     */
    public Callback(Angle theta, Runnable action) {
        this.type = CallbackType.ANGLE;
        this.s = -1.0;
        this.theta = theta;
        this.action = action;
    }

    public CallbackType getType() { return type; }

    public Runnable getAction() { return action; }

    public boolean isTriggered() { return triggered; }

    public void setTriggered(boolean triggered) { this.triggered = triggered; }

    /**
     * Retrieves the target path progression factor.
     *
     * @throws IllegalStateException if this callback is configured as an angular trigger.
     */
    public double getS() {
        if (this.type != CallbackType.DISTANCE) {
            throw new IllegalStateException("Refusing to fetch distance parameter 's' from an " +
                    "ANGLE-type callback!");
        }
        return s;
    }

    /**
     * Retrieves the target triggering angle object.
     *
     * @throws IllegalStateException if this callback is configured as a distance trigger.
     */
    public Angle getTheta() {
        if (this.type != CallbackType.ANGLE) {
            throw new IllegalStateException("Refusing to fetch target angle 'theta' from a " +
                    "DISTANCE-type callback!");
        }
        return theta;
    }
}