package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class GameField {
    // Origin 0,0 at blue 
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json
    private final AprilTagFieldLayout layout;

    public GameField() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public double getDistanceToHub(Pose2d pose) {
        Pose2d hubPose = RobotMap.RED_HUB_POSE;
        return 0;
    }
}
