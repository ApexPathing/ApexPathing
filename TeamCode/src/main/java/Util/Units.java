package Util;

/**
* Class for conversion between distance units and angle units
* @author Sohum Arora - 22985
* @author Xander Haemel - 31616
*/
public class Units {
    public enum DistanceUnits {
        INCHES,
        CENTIMETERS,
        METERS,
        MILLIMETERS,
        FEET
    }

    /**
     * converts a given unit
     * @param unit, object of the DistanceUnits class
     * @param value: value of this unit
     * @return the unit you gave, now converted to inches
     */
    public static double toInches(DistanceUnits unit, double value) {
        switch (unit) {
            case INCHES: return value;
            case CENTIMETERS: return value / 2.54;
            case METERS: return value / 0.0254;
            case MILLIMETERS: return value / 25.4;
            case FEET: return value * 12;
            default: return value;
        }
    }

    /**
     * converts a given unit
     * @param unit, object of the DistanceUnits class
     * @param value: value of this unit
     * @return the unit you gave, now converted to meters
     */
    public static double toMeters(DistanceUnits unit, double value) {
        switch (unit) {
            case METERS: return value;
            case INCHES: return value * 0.0254;
            case CENTIMETERS: return value / 100;
            case MILLIMETERS: return value / 1000;
            case FEET: return value * 0.3048;
            default: return value;
        }
    }
    /**
     * converts a given unit
     * @param unit, object of the DistanceUnits class
     * @param value: value of this unit
     * @return the unit you gave, now converted to centimeters
     */
    public static double toCentimeters(DistanceUnits unit, double value) {
        switch (unit) {
            case CENTIMETERS: return value;
            case INCHES: return value * 2.54;
            case METERS: return value * 100;
            case MILLIMETERS: return value / 10;
            case FEET: return value * 30.48;
            default: return value;
        }
    }
    /**
     * converts a given unit
     * @param unit, object of the DistanceUnits class
     * @param value: value of this unit
     * @return the unit you gave, now converted to millimeters
     */
    public static double toMillimeters(DistanceUnits unit, double value) {
        switch (unit) {
            case MILLIMETERS: return value;
            case INCHES: return value * 25.4;
            case METERS: return value * 1000;
            case CENTIMETERS: return value * 10;
            case FEET: return value * 304.8;
            default: return value;
        }
    }
    /**
     * converts a given unit
     * @param unit, object of the DistanceUnits class
     * @param value: value of this unit
     * @return the unit you gave, now converted to feet
     */
    public static double toFeet(DistanceUnits unit, double value) {
        switch (unit) {
            case FEET: return value;
            case INCHES: return value / 12;
            case METERS: return value / 0.3048;
            case CENTIMETERS: return value / 30.48;
            case MILLIMETERS: return value / 304.8;
            default: return value;
        }

    }
    //default constructor
    public enum AngleUnits {
        DEGREES, RADIANS
    }

    /**
     * converts radians to degrees
     * @param value the value in radians to be converted
     * @return the new value, converted to degrees
     */
    public static double toDegrees(double value) {
        return Math.toDegrees(value);
    }

    /**
     * converts degrees to radians
     * @param value the value in degrees to be converted
     * @return the new value, converted to radians
     */
    public static double toRadians(double value) {
        return Math.toRadians(value);
    }
}
