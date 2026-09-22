package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.IntakeArmSim;
import frc.robot.sim.IntakeCollectorSim;

public class IntakeCollectorSystem extends SubsystemBase {
    private final SparkMax motor;
    private Ultrasonic ultrasonic1;
    private Ultrasonic ultrasonic2;
    private Ultrasonic ultrasonic3;
    private Ultrasonic ultrasonic4;
//not sure where they will be placed so for now I named them by number.

    private final IntakeCollectorSim sim;

    public IntakeCollectorSystem() {
        motor = new SparkMax(RobotMap.COLLECTOR_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(SparkBaseConfig.IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (RobotBase.isSimulation()) {
            sim = new IntakeCollectorSim(motor);
        } else {
            sim = null;
        }

        ultrasonic1 = new Ultrasonic(RobotMap.COLLECTOR_ULTRASONIC1_ID, RobotMap.COLLECTOR_ULTRASONIC_ECO_CHANNEL1);
        ultrasonic2 = new Ultrasonic(RobotMap.COLLECTOR_ULTRASONIC2_ID, RobotMap.COLLECTOR_ULTRASONIC_ECO_CHANNEL2);
        ultrasonic3 = new Ultrasonic(RobotMap.COLLECTOR_ULTRASONIC3_ID, RobotMap.COLLECTOR_ULTRASONIC_ECO_CHANNEL3);
        ultrasonic4 = new Ultrasonic(RobotMap.COLLECTOR_ULTRASONIC4_ID, RobotMap.COLLECTOR_ULTRASONIC_ECO_CHANNEL4);

    }

    public void move(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void periodic() {

    }

    public boolean areThereBalls(){
        if (ultrasonic1.getRangeMM() <= RobotMap.COLLECTOR_ULTRASONIC_MAX_DIS_MM
                || ultrasonic2.getRangeMM() <= RobotMap.COLLECTOR_ULTRASONIC_MAX_DIS_MM
                || ultrasonic3.getRangeMM() <= RobotMap.COLLECTOR_ULTRASONIC_MAX_DIS_MM
                || ultrasonic4.getRangeMM() <= RobotMap.COLLECTOR_ULTRASONIC_MAX_DIS_MM)
            return true;
        else {
            return false;
        }
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}