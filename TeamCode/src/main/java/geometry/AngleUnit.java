package geometry;

/**
 * Enum to represent angle units and provide conversion methods between them.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public enum AngleUnit {
    /** Radian */
    RAD(1.0),

    /** Degree */
    DEG(Math.PI / 180.0);

    private final double radiansPerUnit;

    AngleUnit(double radiansPerUnit) { this.radiansPerUnit = radiansPerUnit; }

    /** Converts a value in this unit to radians. */
    public double toRadians(double value) { return value * radiansPerUnit; }

    /** Converts a value in radians to this unit. */
    public double fromRadians(double radians) { return radians / radiansPerUnit; }
}