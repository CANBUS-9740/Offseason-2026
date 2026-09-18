package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Swerve;

public class GameField {
    // Origin 0,0 at blue 
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json
    private final AprilTagFieldLayout layout;
    private final Swerve swerve;

    public GameField(Swerve swerve) {
        this.swerve = swerve;
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public Translation2d hubPosition(){
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        if(alliance == DriverStation.Alliance.Red){
            return RobotMap.HUB_RED_METERS;
        }
        else{
            return RobotMap.HUB_BLUE_METERS;
        }
    }

    public double calculateTurretAngleToHubDegrees(Pose2d startPose) {
        Translation2d hubPosition = hubPosition();
        double front = hubPosition.getY() - startPose.getY();
        double nextTo = hubPosition.getX() - startPose.getX();

        double alpha = Math.toDegrees(Math.atan2(front, nextTo));
        double beta = startPose.getRotation().getDegrees();
        double target = alpha - beta;

        double shortenedAngle = MathUtil.inputModulus(target,RobotMap.TURRET_MIN_ANGLE, RobotMap.TURRET_MAX_ANGLE);

        return MathUtil.clamp(shortenedAngle,RobotMap.TURRET_MIN_ANGLE, RobotMap.TURRET_MAX_ANGLE);
    }

    public double calculateTurretAngleToHubDegrees() {
        Pose2d robotPose = swerve.getPose();
        return calculateTurretAngleToHubDegrees(robotPose);
    }
}
