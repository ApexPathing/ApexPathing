package paths;

import core.Follower;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;

public class ExampleAutoPath {
    private static final Pose startPose = Pose.zero();

    public GeometryFactory factory;
    public Path testPath;
    public Turn testTurn;
    public String callbackMessage = "Callback not triggered yet";

    public ExampleAutoPath(Follower follower, GeometryFactory.PoseMirror mirror) {
        factory = new GeometryFactory(follower)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG)
                .setPoseMirror(mirror);

        build();
    }

    public void exampleDistanceCallback() { callbackMessage = "Distance callback triggered!"; }

    public void exampleAngularCallback() { callbackMessage = "Angular callback triggered!"; }

    private void build() {
        testPath = factory.path(startPose, // Forward and left curve
                        factory.pose(20, 0),
                        factory.pose(40, 20),
                        factory.pose(45, 25, 120)
                )
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                .addDistanceCallback(0.5, this::exampleDistanceCallback)
                .profiledBuild();

        testTurn = factory.turn(testPath.getEndPose())
                .turnTo(factory.angle(45))
                .addAngularCallback(factory.angle(90), this::exampleAngularCallback)
                .quickBuild();
    }
}