package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tuning phase for determining the translational velocity feedforward constant (kV) for the follower.
 * Manual tuning is not supported for this phase as it is calculated from max velocity.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class VelocityFeedforwardPhase extends TuningPhase {
    private final ElapsedTime timer = new ElapsedTime();
    private double maxVel = 0.0;

    public VelocityFeedforwardPhase(TunerContext context) { super(context); }

    @Override
    public String getPhaseName() { return "Velocity Feedforward"; }

    @Override
    protected boolean manualTuneIsPossible() { return false; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() { timer.reset(); }

    @Override
    protected boolean manualUpdate() { return true; } // Manual tuning is not supported

    @Override
    protected boolean automaticUpdate() {
        context.getFollower().update();
        context.getFollower().getDrivetrain().moveWithVectors(0, 1.0, 0);

        maxVel = Math.max(
                Math.abs(context.getFollower().getVelocity().getY().getIn()), maxVel
        );

        if (timer.milliseconds() > 3000) {
            context.constants.translationalKV = 1.0 / maxVel;
            context.getFollower().stop();
            return true;
        }

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Max Velocity (in/s)", maxVel);
        context.getTelemetry().addData("Calculated kV", context.constants.translationalKV);
    }
}
