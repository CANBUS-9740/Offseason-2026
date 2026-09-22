package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if(alliance == DriverStation.Alliance.Red){
            return RobotMap.HUB_RED_METERS;
        }
        else{
            return RobotMap.HUB_BLUE_METERS;
        }
    }
    public double calculateTurretAngleAlpha(Pose2d startPose) {
        Translation2d hubPosition = hubPosition();
        double front = hubPosition.getY() - startPose.getY();
        double nextTo = hubPosition.getX() - startPose.getX();

        return Math.toDegrees(Math.atan2(front, nextTo));
    }

    public double calculateTurretAngleToHubDegrees(Pose2d startPose) {
        Translation2d hubPosition = hubPosition();
        double front = hubPosition.getY() - startPose.getY();
        double nextTo = hubPosition.getX() - startPose.getX();

        double alpha = Math.toDegrees(Math.atan2(front, nextTo));
        double beta = startPose.getRotation().getDegrees();
        double target = alpha - beta;

        return target % 360; // MathUtil.inputModulus(target,RobotMap.TURRET_MIN_ANGLE, RobotMap.TURRET_MAX_ANGLE);
    }

    public Pair<Double, Double> calculateTurretAndSwerveAnglesToHubDegrees(Pose2d robotPose) {
        double turretAngle = calculateTurretAngleToHubDegrees(robotPose);
        double clampedTurretAngle = MathUtil.clamp(turretAngle, RobotMap.TURRET_MIN_ANGLE, RobotMap.TURRET_MAX_ANGLE);
        if (turretAngle != clampedTurretAngle) {
            return Pair.of(clampedTurretAngle,calculateTurretAngleAlpha(robotPose));
        } else {
            return Pair.of(clampedTurretAngle, robotPose.getRotation().getDegrees());
        }
    }

    public double calculateTurretAngleToHubDegrees() {
        Pose2d robotPose = swerve.getPose();
        return calculateTurretAngleToHubDegrees(robotPose);
    }
}
