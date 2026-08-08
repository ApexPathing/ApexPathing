package tuning;

/**
 * A binary search algorithm that finds a value within a specified range. The search continues until
 * the difference between the last guess and the current guess is less than the specified threshold.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class BinarySearch {
    private final double threshold;
    private double maximum;
    private double minimum;
    private double guess;

    BinarySearch(double minimum, double maximum, double threshold) {
        this.maximum = maximum;
        this.minimum = minimum;
        this.threshold = threshold;
        guess = (maximum + minimum) / 2.0;
    }

    boolean updateGuess(boolean increase) {
        double lastGuess = guess;

        if (increase) { minimum = guess; }
        else { maximum = guess; }

        guess = (maximum + minimum) / 2.0;
        return Math.abs(lastGuess - guess) > threshold;
    }

    double getGuess() { return guess; }
}