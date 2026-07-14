package geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * A class representing a distance that can be easily converted between {@link DistUnit}s.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public final class Dist {
    private final double inches;

    // region Constructors and factory methods

    private Dist(double inches) { this.inches = inches; }

    /** Creates a Distance from a specific unit. */
    public static Dist of(double value, DistUnit unit) { return new Dist(unit.toInches(value)); }

    /** @return the distance value in the requested unit. */
    public double get(DistUnit unit) { return unit.fromInches(this.inches); }

    /** Creates a Distance from inches. */
    public static Dist fromIn(double inches) { return of(inches, DistUnit.IN); }

    /** Creates a Distance from feet. */
    public static Dist fromFt(double feet) { return of(feet, DistUnit.FT); }

    /** Creates a Distance from millimeters. */
    public static Dist fromMm(double millimeters) { return of(millimeters, DistUnit.MM); }

    /** Creates a Distance from centimeters. */
    public static Dist fromCm(double centimeters) {return of(centimeters, DistUnit.CM);}

    /** Creates a Distance from meters. */
    public static Dist fromM(double meters) { return of(meters, DistUnit.M); }

    /** Creates a Distance with a value of 0 inches. */
    public static Dist zero() { return new Dist(0); }

    // endregion
    // region Getters

    /** @return the distance in inches. */
    public double getIn() { return this.inches; }

    /** @return the distance in feet. */
    public double getFt() { return get(DistUnit.FT); }

    /** @return the distance in millimeters. */
    public double getMm() { return get(DistUnit.MM); }

    /** @return the distance in centimeters. */
    public double getCm() { return get(DistUnit.CM); }

    /** @return the distance in meters. */
    public double getM() { return get(DistUnit.M); }

    // endregion
    // region Arithmetic operations

    /** @return a new Distance that is the sum of this Distance and another one. */
    public Dist plus(Dist other) { return of(this.inches + other.inches, DistUnit.IN); }

    /** @return a new Distance that is the difference of this Distance and another one. */
    public Dist minus(Dist other) { return of(this.inches - other.inches, DistUnit.IN); }

    /** @return a new Distance that is this Distance multiplied by a scalar. */
    public Dist times(double scalar) { return of(this.inches * scalar, DistUnit.IN); }

    /** @return a new Distance that is this Distance multiplied by another Distance. */
    public Dist times(Dist other) { return of(this.inches * other.inches, DistUnit.IN); }

    /** @return a new Distance that is this Distance divided by a scalar. */
    public Dist div(double scalar) { return of(this.inches / scalar, DistUnit.IN); }

    /** @return a new Distance that is this Distance divided by another Distance. */
    public Dist div(Dist other) { return of(this.inches / other.inches, DistUnit.IN); }

    /** @return a new Distance that is the absolute value of this Distance. */
    public Dist abs() { return of(Math.abs(this.inches), DistUnit.IN); }

    /** @return a new Distance that is the hypotenuse of this Distance and another one. */
    public Dist hypot(Dist other) { return of(Math.hypot(this.inches, other.inches), DistUnit.IN); }

    // endregion
    // region Other operations and methods

    /** @return a new Distance that is the mirror of this Distance across the origin. */
    public Dist mirror() { return new Dist(-this.inches); }

    /** @return a copy of this Distance */
    public Dist copy() { return new Dist(this.inches); }

    @SuppressLint("DefaultLocale")
    @Override
    @NonNull
    public String toString() { return String.format("%.3f in", this.inches); }

    // endregion
}