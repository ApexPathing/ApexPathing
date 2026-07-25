package core;

import drivetrains.BaseDrivetrainConstants;
import localizers.BaseLocalizerConstants;

/**
 * Interface for your Apex Pathing constants class. You should implement this interface and define
 * drivetrainConstants() and localizerConstants() methods to return your drivetrain and localizer
 * constants for the follower to access.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public interface ApexConstants {
    BaseDrivetrainConstants<?> drivetrainConstants();
    BaseLocalizerConstants<?> localizerConstants();
}