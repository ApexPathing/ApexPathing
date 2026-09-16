/**
 * holy balls i cant spell but im too lazy to go back and fix it all
 * have fun reading ts you guys
 */

package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.List;

import localizers.constants.FusionConstants;
import util.Pose;

/* Fusion localizer that combines odometry reaedings with april tag vision using a kalman filter
 *
 * The kalman filter tracks a 3 element vector {x, y, heading} with a 3x3 covariance matirx P.
 * Your pinpoint drives the "predict" step every loop. april tag detections drive
 * the "update" step whenever at least one tag with a known field position is visible
 * When no tags are visible the filter precits on odometry alone,
 * which allows covariance to grow naturally until a tag is next seen.
 *
 * All positions are in inches and angles are in radians
 *
 * Positive X is forward
 * Positive Y is left
 * Heading increases counter-clockwise
 *
 * */
public class Fusion extends Localizer {

    // Hardware
    private final FusionConstants constants;
    private final GoBildaPinpointDriver pinpoint;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    // Kalman filter state

    /**
     * State estimate: {x, y, head}
     */

    private final double[] state = new double[3];

    /**
     * 3x3 error covariance matrix
     * Indexed as P[row * 3 + col]
     */
    private final double[] P = new double[9];



    // Kalman noise matricies (held as arrays for the same layout as P)


    //Process noise covariance Q - set from constants
    private final double[] Q = new double[9];

    // Measurement noise covariance R - set from constants
    private final double[] R = new double[9];


    // the previosu pinpoint position is stored so we can computee the change
    private double prevOdoX = 0.0;
    private double prevOdoY = 0.0;
    private double prevOdoH = 0.0;

    // Whether we have initialised the previous odometry snapshot
    private boolean odoPrimed = false;

    // True if at least one apriltag update has been recieved since the start
    private boolean everSeenTag = false;

    public Fusion(FusionConstants constants, GoBildaPinpointDriver pinpoint, AprilTagProcessor processor, VisionPortal portal) {
        this.constants      = constants;
        this.pinpoint       = pinpoint;
        this.aprilTagProcessor = processor;
        this.visionPortal   = portal;

        // Build Q and R from constants (diagonal matricies)
        Q[0] = constants.processNoiseX  * constants.processNoiseX;
        Q[4] = constants.processNoiseY  * constants.processNoiseY;
        Q[8] = constants.processNoiseH  * constants.processNoiseH;

        R[0] = constants.visionNoiseX   * constants.visionNoiseX;
        R[4] = constants.visionNoiseY   * constants.visionNoiseY;
        R[8] = constants.visionNoiseH   * constants.visionNoiseH;

        // Initisalise covariance to a large value so the first tag update is trusted heavily
        P[0] = 100.0;
        P[4] = 100.0;
        P[8] = 100.0;

        currentPose = new Pose(0, 0, 0);
        currentVelocity = new Pose(0, 0, 0);
    }


    // Localizer interfcae

    /**
     * Sets the robots known pose (example: at the start of an OpMode)
     * Resets the Kalmans covariance to the configured initial uncertainty
     *
     */


    @Override
    public void setPose(Pose pose) {
        state[0] = pose.getXComponent().getIn();
        state[1] = pose.getYComponent().getIn();
        state[2] = pose.getHeadingComponent().getRad();

        // Resets covariance
        P[0] = constants.initialCovX;
        P[0] = constants.initialCovX;
        P[4] = constants.initialCovY;
        P[8] = constants.initialCovH;
        P[1] = P[2] = P[3] = P[5] = P[6] = P[7] = 0.0;

        // Sync pinpoint to this pose so deltas are zero on the next loop
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,
                state[0], state[1], AngleUnit.RADIANS, 0));
        pinpoint.setHeading(state[2], AngleUnit.RADIANS);

        prevOdoX = state[0];
        prevOdoY = state[1];
        prevOdoH = state[2];
        odoPrimed = true;

        currentPose = pose.copy();
    }

    /**
     * main update loop. call this every iteration of your opdih loop
     *
     * heres a general explanation of whaat happens
     *
     * Fetches latest data from Pinpoint in a bulk read
     * Computes the odometry delta and runs the Kalman "predict" steps
     * If apriltag detections are available, runs the Kalman "update" steps
     * Writes the fused resutl back to currentPose and currentVeloctity
     *
     */
    @Override
    public void update() {
        // First reads the pinpoint
        pinpoint.update();

        double odoX = pinpoint.getPosX(DistanceUnit.INCH);
        double odoY = pinpoint.getPosY(DistanceUnit.INCH);
        double odoH = pinpoint.getHeading(AngleUnit.RADIANS);

        if(!odoPrimed) {
            prevOdoX = odoX;
            prevOdoY = odoY;
            prevOdoH = odoH;
        }




    }



    // This is the "predict" step I was talking about
    // You have this x_{k|k-1} = x_{k-1|k-1} + u_k (x is like P and u is like Q but js are just variables here)
    // u_k is the matrix [dx, dy, dh] from the delta given from odometry
    // Because the state transition Jacobian F = I, we can rewrite this to P[K|K-1} = P_{k-1|k-1} + Q

    private void predict(double dx, double dy, double dh) {
        // Rotate the odometry delta from the robot frame into field frame
        // use the current heading estimate b4 adding it
        double cosH = Math.cos(state[2]);
        double sinH = Math.sin(state[2]);

        // The pinpoint returns in field frame so we dont have to convert it here
        state[0] += dx;
        state[1] += dy;
        state[2] += wrapAngle(state[2] + dh);

        // P = P + Q  (F = I, so F*P*F^T = P
        for (int i = 0; i < 9; i++) {
            P[i] += Q[i];
        }

    }


    // Wraps an angle to the range between (-pi and pi]
    private static double wrapAngle(double rad) {
        while (rad > Math.PI)
            rad -= 2 * Math.PI;
        while (rad < -Math.PI)
            rad += 2 * Math.PI;
        return rad;
    }

    // Had AI write this method here
    private void applyVisionUpdate(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) return;

        for (AprilTagDetection det : detections) {
            // Only use detections with known field locations
            if (det.metadata == null) continue;

            // Compute robot pose from this detection's field-relative data.
            // The VisionPortal SDK populates robotPose when the tag's field position is known.
            if (det.robotPose == null) continue;

            double zx = det.robotPose.getPosition().x;
            double zy = det.robotPose.getPosition().y;
            double zh = det.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

            // Measurement vector
            double[] z = {zx, zy, zh};

            // Innovation: y = z - H*x  (H = I, so y = z - x)
            double[] innov = {
                    z[0] - state[0],
                    z[1] - state[1],
                    wrapAngle(z[2] - state[2])
            };

            // Skip outliers that are wildly inconsistent with the current estimate.
            // This guards against misidentified tags or momentary glitches.
            double innovDist = Math.sqrt(innov[0] * innov[0] + innov[1] * innov[1]);
            if (innovDist > constants.maxVisionJumpInches) continue;
            if (Math.abs(innov[2]) > constants.maxVisionJumpRad) continue;

            // S = P + R  (H = I)
            double[] S = new double[9];
            for (int i = 0; i < 9; i++) S[i] = P[i] + R[i];

            // K = P * S^{-1}   (3×3 invert S then multiply)
            double[] Sinv = invert3x3(S);
            if (Sinv == null) continue; // singular — skip this detection

            double[] K = multiply3x3(P, Sinv);

            // x = x + K * innov
            state[0] += K[0] * innov[0] + K[1] * innov[1] + K[2] * innov[2];
            state[1] += K[3] * innov[0] + K[4] * innov[1] + K[5] * innov[2];
            state[2] = wrapAngle(state[2] + K[6] * innov[0] + K[7] * innov[1] + K[8] * innov[2]);

            // P = (I - K) * P
            double[] IminusK = new double[9];
            IminusK[0] = 1 - K[0];
            IminusK[1] = -K[1];
            IminusK[2] = -K[2];
            IminusK[3] = -K[3];
            IminusK[4] = 1 - K[4];
            IminusK[5] = -K[5];
            IminusK[6] = -K[6];
            IminusK[7] = -K[7];
            IminusK[8] = 1 - K[8];

            double[] newP = multiply3x3(IminusK, P);
            System.arraycopy(newP, 0, P, 0, 9);

            everSeenTag = true;
        }
    }


    // Multiplies two 3×3 row-major matrices
    private static double[] multiply3x3(double[] A, double[] B) {
        double[] C = new double[9];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                double sum = 0;
                for (int k = 0; k < 3; k++) {
                    sum += A[r * 3 + k] * B[k * 3 + c];
                }
                C[r * 3 + c] = sum;
            }
        }
        return C;
    }



    // Inverts a 3x3 row-major matricies using the analytic cofactor method
    private static double[] invert3x3(double[] M) {
        double a = M[0], b = M[1], c = M[2];
        double d = M[3], e = M[4], f = M[5];
        double g = M[6], h = M[7], i = M[8];

        double det = a * (e * i - f * h)
                - b * (d * i - f * g)
                + c * (d * h - e * g);

        if (Math.abs(det) < 1e-10)
            return null;

        double invDet = 1.0 / det;

        return new double[] {
                (e * i - f * h) * invDet,
                -(b * i - c * h) * invDet,
                (b * f - c * e) * invDet,
                -(d * i - f * g) * invDet,
                (a * i - c * g) * invDet,
                -(a * f - c * d) * invDet,
                (d * h - e * g) * invDet,
                -(a * h - b * g) * invDet,
                (a * e - b * d) * invDet
        };
    }




    private static Pose buildPose(double x, double y, double heading) {
        return new Pose(x, y, heading);
    }

    // =========================================================================
    // Diagnostics
    // =========================================================================

    /** @return current Kalman covariance diagonal [Px, Py, Ph] — useful for telemetry */
    public double[] getCovarianceDiagonal() {
        return new double[] { P[0], P[4], P[8] };
    }

    /** @return {@code true} if a tag update has been received at least once */
    public boolean hasSeenTag() {
        return everSeenTag;
    }

    /** Closes the VisionPortal. Call this in your OpMode's {@code stop()} hook. */
    public void close() {
        visionPortal.close();
    }


}
