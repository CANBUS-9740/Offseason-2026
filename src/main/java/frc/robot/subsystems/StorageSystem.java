package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.StorageSim;

public class StorageSystem extends SubsystemBase {

    private final SparkMax generalRollers;
    private final StorageSim sim;

    public StorageSystem() {
        generalRollers = new SparkMax(RobotMap.STORAGE_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        generalRollers.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new StorageSim(generalRollers);
        } else {
            sim = null;
        }
    }

    public void moveGeneralRollers(double speed) {
        generalRollers.set(speed);
    }

    public void stopMotors() {
        generalRollers.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber("StorageMotorOutput", generalRollers.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}