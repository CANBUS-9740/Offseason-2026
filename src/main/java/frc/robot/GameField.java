package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

import java.util.Locale;
import java.util.Optional;

public class GameField {
    // Origin 0,0 at blue 
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json
    private final AprilTagFieldLayout layout;
    private Swerve swerve;




    public GameField(Swerve swerve) {
        this.swerve = swerve;
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public Translation2d hubPosition(){
        Translation2d hubPosition;
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        if(alliance == DriverStation.Alliance.Red){
            hubPosition = RobotMap.HUB_RED_METERS;
        }
        else{
            hubPosition = RobotMap.HUB_BLUE_METERS;
        }
        return hubPosition;
    }

    public double calculateTurretAngleToHubDegrees(){
        double front,nextTo,alpha,beta,target,shortenedAngle,clampedAngle;
        Pose2d robotPose;
        robotPose = swerve.getPose();
        robotPose.getRotation().getDegrees();

        front = hubPosition().getX() - robotPose.getX();
        nextTo = hubPosition().getY() - robotPose.getY();

        alpha = Math.toDegrees(Math.atan(front/nextTo));
        beta = robotPose.getRotation().getDegrees();

        target = alpha+beta;

        shortenedAngle = MathUtil.inputModulus(target,RobotMap.TURRET_MIN_ANGLE,RobotMap.TURRET_MAX_ANGLE);

        clampedAngle = MathUtil.clamp(shortenedAngle,RobotMap.TURRET_MIN_ANGLE,RobotMap.TURRET_MAX_ANGLE);

        return clampedAngle;
    }
}
