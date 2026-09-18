package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {

    private final SparkFlex shootermotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    private Ultrasonic ultrasonic;

    public ShooterSystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        shootermotor = new SparkFlex(RobotMap.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        config.closedLoop
                .p(RobotMap.PITCHER_PID.kP)
                .i(RobotMap.PITCHER_PID.kI)
                .d(RobotMap.PITCHER_PID.kD);

        shootermotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = shootermotor.getClosedLoopController();
        encoder = shootermotor.getEncoder();
        ultrasonic = new Ultrasonic(RobotMap.SHOOTER_ULTRASONIC_ID, RobotMap.SHOOTER_ULTRASONIC_ECO_CHANNEL);


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
}
