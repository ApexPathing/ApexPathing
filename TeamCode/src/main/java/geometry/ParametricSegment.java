package geometry;

/**
 * Represents a purely mathematical, stateless 2D parametric curve.
 * Evaluated strictly using the parameter 't' in the domain [0.0, 1.0].
 *
 * @author DrPixelCat - 7842 alum
 */
public interface ParametricSegment {
    /** @return The (x, y) coordinate at parameter t */
    Vector getPosition(double t);

    /** @return The first derivative (velocity vector) at parameter t */
    Vector getFirstDerivative(double t);

    /** @return The second derivative (acceleration vector) at parameter t */
    Vector getSecondDerivative(double t);
}