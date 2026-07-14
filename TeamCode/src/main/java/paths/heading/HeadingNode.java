package paths.heading;

import geometry.Angle;

/**
 * Represents a heading target at a specific percentage of the path.
 *
 * @author DrPixelCat - 7842 alum
 */
public class HeadingNode implements Comparable<HeadingNode> {
    public final double pct;
    public final Angle target;

    public HeadingNode(double pct, Angle target) {
        this.pct = Math.min(Math.max(pct, 0.0), 1.0);
        this.target = target;
    }

    @Override
    public int compareTo(HeadingNode other) { return Double.compare(this.pct, other.pct); }
}