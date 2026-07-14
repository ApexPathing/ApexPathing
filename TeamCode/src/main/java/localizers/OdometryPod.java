package localizers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper class for 2/3 wheel odometry and drive encoder localizers.
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class OdometryPod {
    private final String name;

    private final double ticksPerInch;
    private final DcMotorEx odometry;

    private int lastTicks;
    private int currentTicks;
    private double deltaTicks;

    public OdometryPod(HardwareMap hardwareMap, String name, double ticksPerInch) {
        this.name = name;
        this.odometry = hardwareMap.get(DcMotorEx.class, this.name);
        this.ticksPerInch = ticksPerInch;
    }

    public String getName() { return this.name; }

    public void update() {
        currentTicks = odometry.getCurrentPosition();
        deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;
    }

    public void reset() {
        lastTicks = 0;
        currentTicks = 0;
        odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** @return the amount of inches the encoder has moved since the last reset */
    public double getInches() { return currentTicks / ticksPerInch; }

    /** @return the amount of inches the encoder has moved since the last loop. */
    public double getDeltaInches() { return deltaTicks / ticksPerInch; }
}

