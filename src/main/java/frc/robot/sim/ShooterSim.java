package frc.robot.sim;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotMap;

public class ShooterSim {

    private final SparkFlexSim shooterMotorSim;
    private final FlywheelSim shooterSim;

    public ShooterSim(SparkFlex shooterMotor) {
        shooterMotorSim = new SparkFlexSim(shooterMotor, RobotMap.SHOOTER_MOTOR);
        shooterSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(RobotMap.SHOOTER_MOTOR, RobotMap.SHOOTER_MOI, RobotMap.SHOOTER_GEAR_RATIO),
                RobotMap.SHOOTER_MOTOR);
    }

    public void update() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double shooterVolts = shooterMotorSim.getAppliedOutput() * batteryVoltage;
        shooterSim.setInputVoltage(shooterVolts);
        shooterSim.update(0.020);
        shooterMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(shooterSim.getAngularVelocityRadPerSec()),
                batteryVoltage,
                0.020);
    }
}
