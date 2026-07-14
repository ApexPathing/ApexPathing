package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Abstract class implemented by all localizer configuration classes
 *
 * <p>
 * When creating a localization configuration, you must extend this class and implement the build()
 * method to return an instance of the corresponding localizer class using your configuration class.
 * Your constants should have a public scope and be initialized with default values.
 * </p>
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseLocalizerConstants<T extends BaseLocalizerConstants<T>> {
    /**
     * Builds and returns an instance of the corresponding localizer class using this configuration.
     */
    public abstract BaseLocalizer<?> build(HardwareMap hardwareMap);
}