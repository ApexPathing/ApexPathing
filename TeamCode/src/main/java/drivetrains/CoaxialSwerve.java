package drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
import geometry.Angle;
import geometry.Dist;

/**
 * Coaxial swerve drivetrain controller
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CoaxialSwerve extends BaseDrivetrain<CoaxialSwerve.Constants> {
    private final static double pi2 = 2 * Math.PI;
    private final static double piOver2 = Math.PI / 2;

    private final CRServo flServo, frServo, blServo, brServo;
    private final AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;
    private final PDSController flSteerController, frSteerController, blSteerController,
            brSteerController;
    private final double voltageToRad; // Radians = voltage * this
    private final double offsetAngleRad; // Radians added to encoder angle to get actual wheel angle

    private double lastFlError = 0, lastFrError = 0, lastBlError = 0, lastBrError = 0;

    public CoaxialSwerve(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.COAXIAL_SWERVE);

        flServo = hardwareMap.get(CRServo.class, constants.flServoName);
        frServo = hardwareMap.get(CRServo.class, constants.frServoName);
        blServo = hardwareMap.get(CRServo.class, constants.blServoName);
        brServo = hardwareMap.get(CRServo.class, constants.brServoName);
        flEncoder = hardwareMap.get(AnalogInput.class, constants.flEncoderName);
        frEncoder = hardwareMap.get(AnalogInput.class, constants.frEncoderName);
        blEncoder = hardwareMap.get(AnalogInput.class, constants.blEncoderName);
        brEncoder = hardwareMap.get(AnalogInput.class, constants.brEncoderName);

        flSteerController = new PDSController(constants.steeringCoefficients);
        frSteerController = new PDSController(constants.steeringCoefficients);
        blSteerController = new PDSController(constants.steeringCoefficients);
        brSteerController = new PDSController(constants.steeringCoefficients);

        voltageToRad = 2 * Math.PI / flEncoder.getMaxVoltage();
        offsetAngleRad = constants.offsetAngle.getRad();
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Coaxial swerve kinematics explanation:
        // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
        double cwTurn = turn * -1; // Clockwise turn angle

        double strafeFront = y + cwTurn * constants.wheelbaseRatio;
        double strafeRear = y - cwTurn * constants.wheelbaseRatio;
        double forwardLeft = x + cwTurn * constants.trackWidthRatio;
        double forwardRight = x - cwTurn * constants.trackWidthRatio;

        double flAngleTarget = Math.atan2(strafeFront, forwardLeft);
        double frAngleTarget = Math.atan2(strafeFront, forwardRight);
        double blAngleTarget = Math.atan2(strafeRear, forwardLeft);
        double brAngleTarget = Math.atan2(strafeRear, forwardRight);

        double flAngle = Angle.normalize(flEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double frAngle = Angle.normalize(frEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double blAngle = Angle.normalize(blEncoder.getVoltage() * voltageToRad + offsetAngleRad);
        double brAngle = Angle.normalize(brEncoder.getVoltage() * voltageToRad + offsetAngleRad);

        double flError = wrapError(flAngleTarget, flAngle);
        double frError = wrapError(frAngleTarget, frAngle);
        double blError = wrapError(blAngleTarget, blAngle);
        double brError = wrapError(brAngleTarget, brAngle);

        // If error > 90 degrees, reverse wheel direction and subtract 180 from target angle
        double flPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardLeft, 2));
        if (Math.abs(flError) > piOver2) {
            flPower *= -1;
            flError -= Math.copySign(Math.PI, flError);
        }
        double frPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardRight, 2));
        if (Math.abs(frError) > piOver2) {
            frPower *= -1;
            frError -= Math.copySign(Math.PI, frError);
        }
        double blPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardLeft, 2));
        if (Math.abs(blError) > piOver2) {
            blPower *= -1;
            blError -= Math.copySign(Math.PI, blError);
        }
        double brPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardRight, 2));
        if (Math.abs(brError) > piOver2) {
            brPower *= -1;
            brError -= Math.copySign(Math.PI, brError);
        }

        if (flError != lastFlError) {
            flServo.setPower(flSteerController.calculate(flError));
            lastFlError = flError;
        }
        if (frError != lastFrError) {
            frServo.setPower(frSteerController.calculate(frError));
            lastFrError = frError;
        }
        if (blError != lastBlError) {
            blServo.setPower(blSteerController.calculate(blError));
            lastBlError = blError;
        }
        if (brError != lastBrError) {
            brServo.setPower(brSteerController.calculate(brError));
            lastBrError = brError;
        }

        setPowers(flPower, frPower, blPower, brPower);
    }

    /** @return the error wrapped to the range [-pi, pi] to ensure the shortest path is taken */
    private double wrapError(double target, double current) {
        double errorRaw = target - current;
        return errorRaw - pi2 * Math.round(errorRaw / pi2);
    }

    /** Configuration class for Coaxial Swerve drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        public String flServoName, frServoName, blServoName, brServoName;
        public String flEncoderName, frEncoderName, blEncoderName, brEncoderName;

        public PDSCoefficients steeringCoefficients = new PDSCoefficients();

        public Angle offsetAngle = Angle.zero(); // Encoder value facing forward position
        public Dist wheelbase = Dist.fromIn(14); // Front pod to back pod spacing
        public Dist trackWidth = Dist.fromIn(14); // Left pod to right pod spacing
        public Dist diagonalDist;
        public double wheelbaseRatio; // Wheelbase / diagonal distance
        public double trackWidthRatio; // Track width / diagonal distance

        @Override
        public CoaxialSwerve build(HardwareMap hardwareMap) {
            diagonalDist = wheelbase.hypot(trackWidth);
            wheelbaseRatio = wheelbase.div(diagonalDist).getIn();
            trackWidthRatio = trackWidth.div(diagonalDist).getIn();

            if (flMotorConfig == null || frMotorConfig == null || blMotorConfig == null ||
                    brMotorConfig == null) {
                throw new IllegalArgumentException(
                        "All 4 motor configs must be provided for a coaxial swerve drivetrain."
                );
            }
            if (flServoName == null || frServoName == null || blServoName == null ||
                    brServoName == null) {
                throw new IllegalArgumentException(
                        "All 4 servo names must be provided for a coaxial swerve drivetrain."
                );
            }
            if (flEncoderName == null || frEncoderName == null || blEncoderName == null ||
                    brEncoderName == null) {
                throw new IllegalArgumentException(
                        "All 4 encoder names must be provided for a coaxial swerve drivetrain."
                );
            }

            return new CoaxialSwerve(this, hardwareMap);
        }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(Motor Motor) {
            this.flMotorConfig = Motor;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(Motor Motor) {
            this.frMotorConfig = Motor;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(Motor Motor) {
            this.blMotorConfig = Motor;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(Motor Motor) {
            this.brMotorConfig = Motor;
            return this;
        }

        /** Sets the names of the steering servos. */
        public Constants setServoNames(String fl, String fr, String bl, String br) {
            this.flServoName = fl;
            this.frServoName = fr;
            this.blServoName = bl;
            this.brServoName = br;
            return this;
        }

        /** Sets the names of the steering encoders. */
        public Constants setEncoderNames(String fl, String fr, String bl, String br) {
            this.flEncoderName = fl;
            this.frEncoderName = fr;
            this.blEncoderName = bl;
            this.brEncoderName = br;
            return this;
        }

        /** Sets the coefficients for the steering PDS controllers. */
        public Constants setSteeringCoefficients(PDSCoefficients coefficients) {
            this.steeringCoefficients = coefficients;
            return this;
        }

        /** Sets the encoders reported angle when the wheel is facing forward. */
        public Constants setOffsetAngle(Angle offsetAngle) {
            this.offsetAngle = offsetAngle;
            return this;
        }

        /** Sets the wheelbase of the chassis (front to back pod wheel spacing). */
        public Constants setWheelbase(Dist wheelbase) {
            this.wheelbase = wheelbase; return this;
        }

        /** Sets the track width of the chassis (left to right pod wheel spacing). */
        public Constants setTrackWidth(Dist trackWidth) {
            this.trackWidth = trackWidth; return this;
        }
    }
}