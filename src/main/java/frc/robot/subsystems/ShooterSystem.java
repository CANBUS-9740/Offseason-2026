package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.ShooterSim;

public class ShooterSystem extends SubsystemBase {

    private final SparkFlex shootermotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private Ultrasonic ultrasonic;

    private final ShooterSim sim;
    public ShooterSystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        shootermotor = new SparkFlex(RobotMap.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        config.closedLoop
                .p(RobotMap.SHOOTER_PID.kP)
                .i(RobotMap.SHOOTER_PID.kI)
                .d(RobotMap.SHOOTER_PID.kD);

        shootermotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pidController = shootermotor.getClosedLoopController();
        encoder = shootermotor.getEncoder();
        ultrasonic = new Ultrasonic(RobotMap.SHOOTER_ULTRASONIC_ID, RobotMap.SHOOTER_ULTRASONIC_ECO_CHANNEL);

        if (RobotBase.isSimulation()) {
            sim = new ShooterSim(shootermotor);
        } else {
            sim = null;
        }
    }

    public void set(double speed) {
        shootermotor.set(speed);
    }

    public void stop() {
        shootermotor.stopMotor();
    }

    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    public void setRotateAtVelocity(double velocityRPM) {
        double ff = velocityRPM / RobotMap.MAX_SHOOTER_VELOCITY_RPM;
        pidController.setSetpoint(velocityRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public boolean areThereBalls() {
        if (ultrasonic.getRangeMM() <= RobotMap.SHOOTER_ULTRASONIC_MIN_DIS_MM) {
            return false;
        } else {
            return true;
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor RPM", getVelocityRPM());
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
