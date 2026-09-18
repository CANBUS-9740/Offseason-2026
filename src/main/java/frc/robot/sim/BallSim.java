package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GameField;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import org.dyn4j.geometry.Vector3;

import java.util.function.Supplier;

public class BallSim {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Pose2d> hubPoseSupplier;
    private final Field2d field;
    private final NetworkTable table;
    private SimulatedBall ball;
    private final FiringDisplay firingDisplay;

    public BallSim(Swerve swerve, GameField gameField) {
        robotPoseSupplier = swerve::getPose;
        hubPoseSupplier = ()-> RobotMap.RED_HUB_POSE;
        field = swerve.getField();

        table = NetworkTableInstance.getDefault().getTable("ball");
        firingDisplay = new FiringDisplay();
        SmartDashboard.putData("FiringDisplay", firingDisplay);

        firingDisplay.setHubPosition(new Pose2d(6, 0, Rotation2d.kZero));
    }

    public void launchBall(Translation3d launchPosition, double shooterDirectionDegrees, double shooterRpm, double firingAngleDegrees) {
        double firingVelocity = shooterRpm * 2 * Math.PI * RobotMap.SHOOTER_WHEEL_RADIUS_METERS / 60;
        double shooterDirection = Math.toRadians(shooterDirectionDegrees);
        double firingAngle = Math.toRadians(firingAngleDegrees);

        double velX = firingVelocity * Math.sin(firingAngle) * Math.cos(shooterDirection);
        double velY = firingVelocity * Math.sin(firingAngle) * Math.sin(shooterDirection);
        double velZ = firingVelocity * Math.cos(firingAngle);

        Vector3 position = new Vector3(launchPosition.getX(), launchPosition.getY(), launchPosition.getZ());
        Vector3 velocity = new Vector3(velX, velY, velZ);
        Vector3 acceleration = new Vector3(0, 0, -9.8);

        FieldObject2d fieldObject = field.getObject("Ball");
        SimulatedBall ball = new SimulatedBall(fieldObject, table);

        firingDisplay.resetTrajectory();
        firingDisplay.setBallPosition(position);

        ball.setPosition(position);
        ball.setVelocity(velocity);
        ball.setAcceleration(acceleration);

        this.ball = ball;
    }

    public void update() {
        firingDisplay.setRobotPosition(robotPoseSupplier.get());
        firingDisplay.setHubPosition(hubPoseSupplier.get());

        if (ball != null) {
            ball.update(0.02);
            firingDisplay.setBallPosition(ball.getPosition());
        }
    }
}
