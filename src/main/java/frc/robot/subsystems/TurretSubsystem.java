package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput forwardLimitSwitch;
    private final DigitalInput middleLimitSwitch1;
    private final DigitalInput middleLimitSwitch2;
    private final PIDController pidController;
    private final SparkMaxConfig config;


    public TurretSubsystem(){
        motor = new SparkMax(RobotMap.TURRET_MOTOR_ID , SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        encoder = motor.getEncoder();
        forwardLimitSwitch = new DigitalInput(RobotMap.TURRET_FORWARD_LIMIT_SWITCH_ID);
        middleLimitSwitch1 = new DigitalInput(RobotMap.TURRET_MIDDLE_LIMIT_SWITCH_1_ID);
        middleLimitSwitch2 = new DigitalInput(RobotMap.TURRET_MIDDLE_LIMIT_SWITCH_2_ID);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.closedLoop.pid(RobotMap.TURRET_PID.kP,RobotMap.TURRET_PID.kI,RobotMap.TURRET_PID.kD);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);

        pidController = new PIDController(0,0,0);

    }

    public void setMotor(double speed){
        motor.set(speed);
    }

    public void stopMotor(){
        motor.stopMotor();
    }


    public void setMoveToPosition(double positionDegrees){
       pidController.setSetpoint(positionDegrees);
    }

    public double getPositionDegrees(){
        return encoder.getPosition() * 360;
    }

    public boolean isAlignedForward(){
        return forwardLimitSwitch.get();
    }

    public boolean isAlignedMiddle1(){
        return middleLimitSwitch1.get();
    }

    public boolean isAlignedMiddle2(){
        return middleLimitSwitch2.get();
    }

    public void periodic(){
        SmartDashboard.putNumber("turretPositionDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("turretForwardLimitSwitch" , isAlignedForward());
        SmartDashboard.putBoolean("turretLimitSwitch1" , isAlignedMiddle1());
        SmartDashboard.putBoolean("turretLimitSwitch2" , isAlignedMiddle2());

    }

}
