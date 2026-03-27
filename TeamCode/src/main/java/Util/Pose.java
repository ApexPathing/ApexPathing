package Util;

import static java.lang.Math.*;

/**
 * Class to represent a position in 2D space consisting of x-y coordinates and a heading orientation.
 *
 * Provides methods for arithmetic functions with positions.
 *
 * @author Sohum Arora
 */
public class Pose {

    private double x;
    private double y;
    private double heading;

    // Constructors
    public Pose() {
        this(0.0, 0.0, 0.0);
    }

    public Pose(double x, double y) {
        this(x, y, 0.0);
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Getters / Setters
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setHeading(double heading) { this.heading = heading; }

    // Distance
    public double distanceTo(Pose other) {
        return sqrt(distanceSquaredFrom(other));
    }

    public double distanceFrom(Pose other) {
        return distanceTo(other);
    }

    public double distanceSquaredFrom(Pose other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return dx * dx + dy * dy;
    }

    // Convert to vector
    public Vector asVector() {
        return new Vector(x, y);
    }

    // Arithmetic
    public Pose add(Pose other) {
        return new Pose(
                x + other.x,
                y + other.y,
                heading + other.heading
        );
    }

    public Pose add(Vector vec) {
        return new Pose(
                x + vec.getX(),
                y + vec.getY(),
                heading
        );
    }

    public Pose subtract(Pose other) {
        return new Pose(
                x - other.x,
                y - other.y,
                heading - other.heading
        );
    }

    public Pose subtract(Vector vec) {
        return new Pose(
                x - vec.getX(),
                y - vec.getY(),
                heading
        );
    }

    public Pose multiply(double scalar) {
        return new Pose(
                x * scalar,
                y * scalar,
                heading
        );
    }

    public Pose div(double scalar) {
        return new Pose(
                x / scalar,
                y / scalar,
                heading
        );
    }

    public Pose negate() {
        return new Pose(
                -x,
                -y,
                -heading
        );
    }

    // Reflection
    public Pose reflectX(double at) {
        this.y = 2 * at - this.y;
        this.heading = normalize(-heading);
        return this;
    }

    public Pose withReflectedX(double at) {
        return new Pose(
                x,
                2 * at - y,
                normalize(-heading)
        );
    }

    public Pose reflectY(double at) {
        this.x = 2 * at - this.x;
        this.heading = normalize(PI - heading);
        return this;
    }

    public Pose withReflectedY(double at) {
        return new Pose(
                2 * at - x,
                y,
                normalize(PI - heading)
        );
    }


    public Pose rotate(double theta) {
        return rotate(theta, theta);
    }


    public Pose rotate(double theta, double headingTheta) {
        double newX = x * cos(theta) - y * sin(theta);
        double newY = x * sin(theta) + y * cos(theta);

        this.x = newX;
        this.y = newY;
        this.heading += headingTheta;

        return this;
    }

    public Pose rotated(double theta) {
        return rotated(theta, theta);
    }

    public Pose rotated(double theta, double headingTheta) {
        return new Pose(
                x * cos(theta) - y * sin(theta),
                x * sin(theta) + y * cos(theta),
                heading + headingTheta
        );
    }

    // Utility
    public boolean nearPose(Pose other, double threshold) {
        return distanceFrom(other) < threshold;
    }

    /**
     * duplicate a pose
     * @return copied pose
     */

    public Pose copy() {
        return new Pose(x, y, heading);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + heading + ")";
    }

    public String debug() {
        return "Pose { x: " + x + ", y: " + y + ", heading: " + heading + " }";
    }


    // Angle normalization
    public static double normalize(double angle) {
        double a = angle % (2 * PI);
        if (a > PI) a -= 2 * PI;
        if (a <= -PI) a += 2 * PI;
        return a;
    }
}