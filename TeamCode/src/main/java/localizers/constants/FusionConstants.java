package localizers.constants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import localizers.Fusion;
public class FusionConstants extends LocalizerConstants {


    public String pinpointDeviceName = "pinpoint";


    public String camName = "cam";


    public double xOffset = 0.0;

    public double yOffset = 0.0;

    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public GoBildaPinpointDriver.EncoderDirection xDirection =  GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;


    // Tag vision bull shi

    public boolean drawTagOverlay = true;

    public int camResolutionWidth = 640;
    public int camResolutionHeight = 480;



    // Uhhh attenot at a Kalman filter

    //  Kalman filter (odomerttry set up)
    /** initial covariance (variance) for the x, y and heading values
     * The filter will trust the first tag update heavily so keep this in mind*/

    public double initialCovX = 100.0;

    public double initialCovY = 100.0;

    public double initialCovH = 1.0;

    // The amount of uncertanty per update cycle from the ofometry pods

    /**
     * normal deviation of the x pod noise per update cycle in inches
     * raise this value if the pinpoint drifts noticeably over a test
     */
    public double processNoiseX = 0.05;
    public double processNoiseY = 0.05;
    public double processNoiseH = 0.005;


    //  Kalman filter camera noise (apriltag detection)

    /**
     * Normal method of the april tag measurement in inches
     * raise if your camera tends to give noisy readings
     */

    // Typical starting values are from 0.5 to 2.0
    public double visionNoiseX = 1.0;

    // Typical starting values are from 0.5 to 2.0
    public double visionNoiseY = 1.0;

    // Vision heading is often noiser than position for sum reason
    // Start at 0.05 and slowly increase as needed
    public double visionNoiseH = 0.05;


    // Outlier rejection

    /**
     * maximum allowable Euclidean distance between the current state estimate
     * and a new Apriltag position measurement before the detection is rejected and used as an outlier\
     * increase if your robot teleports on panels lmfao
     * decrease if faulty detections cause jumps
     * start at 24.0 (2 feets)
     */
    
    public double maxVisionJumpInches = 24;
    
    // Same thing but for heading
    public double maxVisionJumpRad = Math.toRadians(45);
    
    // Here is your big ahh constructor :/


    @Override
    public Fusion build(HardwareMap hardwareMap) {
        // pinpoint init
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointDeviceName);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(encoderResolution);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.resetPosAndIMU();

        // tag setup
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder();
        AprilTagProcessor aprilTagProcessor = tagBuilder.build();

        // vision  setup
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, camName))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(camResolutionWidth, camResolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(drawTagOverlay)
                .build();

        return new Fusion(this, pinpoint, aprilTagProcessor, visionPortal);
    }

    // ALl your set up and return and get and other thingy methosd that dont need to be explained lwk :)
    public FusionConstants setPinpointDeviceName(String name){
        this.pinpointDeviceName = name;
        return this;
    }

    public FusionConstants setCameraName(String name){
        this.camName = name;
        return this;
    }

    public FusionConstants setXOffset(double xOffset) {
        this.xOffset = xOffset; 
        return this;
    }

    public FusionConstants setYOffset(double yOffset) {
        this.yOffset = yOffset;
        return this;
    }

    // Weird thing here,
    // Had to make parameter the GoBildaPinpointDriver.GoBildaOdometryPods instead of a double
    public FusionConstants setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods resolution) {
        this.encoderResolution = resolution;
        return this;
    }

    public FusionConstants setXDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.xDirection = direction; return this;
    }

    public FusionConstants setYDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.yDirection = direction; return this;
    }

    public FusionConstants setDrawTagOverlay(boolean draw) {
        this.drawTagOverlay = draw; return this;
    }

    public FusionConstants setCameraResolution(int width, int height) {
        this.camResolutionWidth  = width;
        this.camResolutionHeight = height;
        return this;
    }

    public FusionConstants setInitialCovariance(double covX, double covY, double covH) {
        this.initialCovX = covX;
        this.initialCovY = covY;
        this.initialCovH = covH;
        return this;
    }

    public FusionConstants setProcessNoise(double x, double y, double h) {
        this.processNoiseX = x;
        this.processNoiseY = y;
        this.processNoiseH = h;
        return this;
    }

    public FusionConstants setVisionNoise(double x, double y, double h) {
        this.visionNoiseX = x;
        this.visionNoiseY = y;
        this.visionNoiseH = h;
        return this;
    }


    public FusionConstants setOutlierThresholds(double maxInches, double maxRad) {
        this.maxVisionJumpInches = maxInches;
        this.maxVisionJumpRad    = maxRad;
        return this;
    }
}
