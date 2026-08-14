package core;

import geometry.Pose;
import paths.movements.Path;

/**
 * Optional observer for follower visualization and diagnostics integrations.
 *
 * <p>The production default is a no-op, allowing desktop tools to observe the follower without
 * adding their logging libraries to the Robot Controller APK.</p>
 */
public interface FollowerDiagnostics {
    /** Publishes the follower's latest localized pose. */
    void recordPose(Pose pose);

    /** Publishes the path that the follower has started executing. */
    void recordCurrentPath(Path path);

    /** Clears the active path when the follower stops or begins a non-path movement. */
    void clearCurrentPath();
}
