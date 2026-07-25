package paths.constraint;

import geometry.Dist;

/**
 * A constraint that applies to the robot's translational movement along a path, such as
 * translational velocity or translational acceleration.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TranslationalConstraint implements PathConstraint {
    private double s;
    private final Type type;
    private final double value_in;

    public TranslationalConstraint(double s, Type type, Dist value) {
        this.s = s;
        this.type = type;
        this.value_in = value.getIn();
    }

    @Override
    public double getS() { return s; }

    @Override
    public void setS(double s) { this.s = s; }

    @Override
    public Type getType() { return type; }

    @SuppressWarnings("PublicMethodNotExposedInInterface")
    public double getValueIn() { return value_in; }
}