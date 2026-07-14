package core;

import drivetrains.BaseDrivetrainConstants;
import localizers.BaseLocalizerConstants;

/**
 * Base class for your Apex Pathing constants class. You should extend this class and implement the
 * drivetrainConstants() and localizerConstants() methods to return your drivetrain and localizer
 * constants for the follower to access.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public abstract class ApexConstants {
    public abstract BaseDrivetrainConstants<?> drivetrainConstants();

    public abstract BaseLocalizerConstants<?> localizerConstants();
}