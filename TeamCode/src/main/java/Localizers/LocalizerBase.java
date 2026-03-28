package Localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Util.Pose;
import Util.Vector;

/**
 *@author Sohum Arora 22985 Paraducks
 *@date 3/19/26
 */

public abstract class LocalizerBase {
    // Add the @Pathing annotation stuff here

    // Stores the last position so that way we can calculate velocity
    public Pose lastPosition = new Pose(0.0, 0.0, 0.0);
    // Stores the last velocity so that way we can calculate acceleration
    public Vector lastVelocity = new Vector(0.0, 0.0);
    public Pose currentPosition = new Pose(0.0, 0.0, 0.0);
    public Vector currentVelocity = new Vector(0.0, 0.0);
    public Vector currentAcceleration = new Vector(0.0, 0.0);


    public Pose getPose() {
        return currentPosition;
    }


    public Pose getVelocity() {
        return new Pose(currentVelocity.getX(), currentVelocity.getY(), 0.0);
    }

    /**
     * This initializes the localizer's hardware map or whatever else needed
     */
    public abstract void initLocalizer(HardwareMap hardwareMap);

    /**
     * This is the update class called when your localizer is attached using the @Pathing annotation
     */

    public abstract void update();

    /**
     *A [Pose] to set the location of the localizer to
     */

    public abstract void setPose(Pose pose);
}
