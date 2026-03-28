package Followers;




import Drivetrains.MecanumDrive;
import Localizers.PinpointLocalizer;
import Util.Pose;
import Util.Vector;

public class P2PFollower extends Follower{

    private final MecanumDrive drive;
    private final PinpointLocalizer localizer;

    private Pose currentPose;
    private Pose targetPose;

     double translationalKp = 0.03;
     double headingKp = 0.5;

     double translationalTolerance = 1.0;
     double headingTolerance = 0.05;

     double maxPower = 1.0;
     double minPower = 0.05;

     boolean isBusy = false;



    public P2PFollower(MecanumDrive drive, PinpointLocalizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setTarget(Pose target) {
        this.targetPose = target;
        this.isBusy = true;
    }

    public void update() {
        localizer.update();
        currentPose = localizer.getPose();

        if (currentPose == null || targetPose == null) {
            return;
        }

        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        double dist = Math.hypot(dx, dy);
        double headingError = normalizeAngle(targetPose.getHeading() - currentPose.getHeading());


        if (dist < translationalTolerance && Math.abs(headingError) < headingTolerance) {
            drive.botCentricDrive(0, 0, 0);
            isBusy = false;
            return;
        }

        Vector error = new Vector(dx, dy);
        error.rotateVec(-currentPose.getHeading());

        double x = error.getX() * translationalKp;
        double y = error.getY() * translationalKp;
        double turn = headingError * headingKp;

        double mag = Math.hypot(x, y);
        if (mag > maxPower) {
            x /= mag;
            y /= mag;
        }

        if (mag > 0) {
            x = applyMinPower(x);
            y = applyMinPower(y);
        }

        turn = clip(turn, -maxPower, maxPower);
        drive.botCentricDrive(x, y, turn);
    }

    public boolean isBusy() {
        return isBusy;
    }

    public Pose getPose() {
        return currentPose;
    }

    private double applyMinPower(double val) {
        if (Math.abs(val) < minPower) {
            return Math.signum(val) * minPower;
        }
        return val;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public void setTargetPose(Pose newTargetPose) {
        targetPose = newTargetPose;
    }

    @Override
    public Pose getCurrentTargetPose() {
        return targetPose;
    }

    /**
     * Method to get translational error in x component
     */
    @Override
    public Vector getXVector() {
        if (currentPose == null || targetPose == null){
            return new Vector(0.0, 0.0);
        }
        double dx = targetPose.getX() - currentPose.getX();
        return new Vector(dx * translationalKp, 0.0);
    }

    /**
     * Method to get translational error in y component
     */
    @Override
    public Vector getYVector() {
        if (currentPose == null || targetPose == null){
            return new Vector(0.0, 0.0);
        }
        double dy = targetPose.getY() - currentPose.getY();
        return new Vector(0.0, dy * translationalKp);
    }
}
