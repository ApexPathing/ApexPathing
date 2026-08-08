package localizers.util;

/**
 * An interface for all 1 dimensional data filters to use
 */
public interface DataFilter {
    public FilterState update(double measurement);
    public void reset();
}
