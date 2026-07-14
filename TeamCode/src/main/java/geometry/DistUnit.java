package geometry;

/**
 * Enum to represent distance units and provide conversion methods between them.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public enum DistUnit {
    /** Inch */
    IN(1.0),
    /** Foot */
    FT(12.0),
    /** Millimeter */
    MM(1.0 / 25.4),
    /** Centimeter */
    CM(1.0 / 2.54),
    /** Meter */
    M(1.0 / 0.0254);

    private final double inchesPerUnit;

    DistUnit(double inchesPerUnit) { this.inchesPerUnit = inchesPerUnit; }

    /** Converts a value in this unit to inches. */
    public double toInches(double value) { return value * inchesPerUnit; }

    /** Converts a value in inches to this unit. */
    public double fromInches(double inches) { return inches / inchesPerUnit; }
}