package Localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Util.Pose;


/**
 * GoBilda Pinpoint odometry localizer
 * @author Sohum Arora 22985
 * TODO: Add StartingPose logic
 */
public class PinpointLocalizer extends LocalizerBase{

    private final HardwareMap hardwareMap;
    private final String deviceName;
    public double xOffset;
    public double yOffset;

    private GoBildaPinpointDriver pinpoint;
    private Pose startPose;

    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName, double xOffset, double yOffset) {
        this.hardwareMap = hardwareMap;
        this.deviceName = deviceName;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, 0.0, 0.0);
    }

    public void init(Pose startPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        this.startPose = startPose.copy();
    }

    public void init() {
        this.init(new Pose());
    }

    @Override
    public void update() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        lastPosition = currentPosition;
        currentPosition = new Pose(
                pos.getX(DistanceUnit.INCH) + startPose.getX(),
                pos.getY(DistanceUnit.INCH) + startPose.getY(),
                pinpoint.getHeading(AngleUnit.RADIANS) + startPose.getHeading()
        );

        currentVelocity.setX((currentPosition.getX() - lastPosition.getX()));
        currentVelocity.setY((currentPosition.getY() - lastPosition.getY()));
        currentVelocity.setTheta( (currentPosition.getHeading() - lastPosition.getHeading()));
    }

    @Override
    public Pose getPose() {
        return currentPosition;
    }

    @Override
    public Pose getVelocity() {
        return new Pose(currentVelocity.getX(), currentVelocity.getY(), 0.0);
    }

    @Override
    public void setPose(Pose pose) {
        currentPosition = pose;
        lastPosition = pose;
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        pose.getX(),
                        pose.getHeading(),
                        AngleUnit.RADIANS,
                        pose.getHeading()
                )
        );
    }

    @Override
    public void initLocalizer(HardwareMap hardwareMap) {
        init();
    }
}