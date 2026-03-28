package Util;

import static java.lang.Math.*;

/**
 * Class to represent a 2D directional vector consisting of formats for both x-y and r-theta values.
 *
 * Provides methods for arithmetic using vectors as well as for transformations.
 *
 * @author Sohum Arora
 * @author Xander Haemel - 31616 - 404 Not Found
 */
public class Vector {

    private double x;
    private double y;
    //default constructor
    public Vector() {
        this(0.0, 0.0);
    }
    //
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }


    public static Vector fromPolar(double r, double theta) {
        return new Vector(r * cos(theta), r * sin(theta));
    }

    /**
     * gets the x component of the vector
     * @return the x
     */
    public double getX() { return x; }
    /**
     * gets the y component of the vector
     * @return the y
     */
    public double getY() { return y; }


    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    /**
     * gets the magnitude of the vector
     * @return
     */
    public double getMagnitude() {
        return hypot(x, y);
    }

    public void setMagnitude(double value) {
        double ang = getTheta();
        this.x = value * cos(ang);
        this.y = value * sin(ang);
    }


    public double getTheta() {
        return atan2(y, x);
    }

    public void setTheta(double value) {
        double r = getMagnitude();
        this.x = r * cos(value);
        this.y = r * sin(value);
    }


    public double getXComponent() { return x; }
    public double getYComponent() { return y; }

    public double dotProduct(Vector other) {
        return this.x * other.x + this.y * other.y;
    }

    public double crossProduct(Vector other) {
        return (x * other.y) - (y * other.x);
    }


    public Vector rotateVec(double ang) {
        return fromPolar(getMagnitude(), normalizeAngle(getTheta() + ang));
    }

    public Vector rotatedVec(double ang) {
        setTheta(normalizeAngle(getTheta() + ang));
        return this;
    }

    public Pose asPose() {
        return new Pose(x, y);
    }

    public Vector normalize() {
        double mag = getMagnitude();
        if (mag > 1e-9) {
            return this.div(mag);
        }
        return new Vector(0.0, 0.0);
    }

    public Vector add(Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector subtract(Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public Vector multiply(double scalar) {
        return new Vector(x * scalar, y * scalar);
    }

    public Vector div(double scalar) {
        return new Vector(x / scalar, y / scalar);
    }

    public Vector negate() {
        return multiply(-1.0);
    }


    public Vector copy() {
        return new Vector(x, y);
    }


    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }

    public String debug() {
        return "Vector <x: " + x + ", y: " + y + ">, <magnitude: " + getMagnitude() + ", θ: " + getTheta() + ">";
    }
    private static double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
        return angle;
    }
}