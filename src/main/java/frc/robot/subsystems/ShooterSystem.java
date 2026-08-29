package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkFlex shootermotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;


    public ShooterSystem(){
        SparkFlexConfig config;
        config = new SparkFlexConfig();
        shootermotor = new SparkFlex(RobotMap.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless); //didnt write to me if you want brushless or not
        config.encoder
                .positionConversionFactor(0)
                .velocityConversionFactor(0);
        config.closedLoop
                .p(RobotMap.SHOOTER_PIDF.p)
                .i(RobotMap.SHOOTER_PIDF.i)
                .d(RobotMap.SHOOTER_PIDF.d); // I have problem with the ff it just doesnt work no matter what i tried
        config.closedLoop.feedForward
                        .kV(RobotMap.SHOOTER_PIDF.f); //i found this but i dont know if its works never used it


        shootermotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //didnt said what you want here
        pidController = shootermotor.getClosedLoopController();
        encoder = shootermotor.getEncoder();


    }
    public void set(double speed){
        shootermotor.set(speed);
    }
    public void stop(){
        shootermotor.stopMotor();
    }

    public double getVelocityRPM(){
        return encoder.getVelocity(); // im pretty sure thats what u wanted

    }
    public void setRotateAtVelocity(double velocityRPM){
        pidController.setSetpoint(velocityRPM, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor RPM", getVelocityRPM());
    }
}
