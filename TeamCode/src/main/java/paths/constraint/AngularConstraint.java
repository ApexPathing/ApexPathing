package paths.constraint;

import geometry.Angle;

/**
 * A constraint that applies to the robot's angular movement along a path, such as angular velocity
 * or angular acceleration.
 *
 * @author DrPixelCat - 7842 alum
 */
public class AngularConstraint implements PathConstraint {
    private double s;
    private final ConstraintType type;

    private final double value_rad;

    public AngularConstraint(double s, ConstraintType type, Angle value) {
        this.s = s;
        this.type = type;
        this.value_rad = value.getRad();
    }

    @Override
    public double getS() { return s; }

    @Override
    public void setS(double s) { this.s = s; }

    @Override
    public ConstraintType getType() { return type; }

    public double getValueRad() { return value_rad; }
}