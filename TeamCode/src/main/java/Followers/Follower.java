package Followers;

import Util.Pose;
import Util.Vector;

public abstract class Follower {
    //declerations
    private boolean followerIsBusy;


    /**
     * sets the follower to a new target Pose
     * @param targetPose the new pose to move to
     */
    public abstract void setTargetPose(Pose targetPose);

    /**
     * gets the current target Pose
     */
    public abstract Pose getCurrentTargetPose();

    /**
     * x vector getter
     * @return the x vector
     */
    public abstract Vector getXVector();
    /**
     * y vector getter
     * @return the y vector
     */
    public abstract Vector getYVector();




}
