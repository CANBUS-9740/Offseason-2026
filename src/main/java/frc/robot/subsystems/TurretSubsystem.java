package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput forwardLimitSwitch;
    private final SparkLimitSwitch middleLimitSwitchForward;
    private final SparkLimitSwitch middleLimitSwitchBackward;
    private final SparkClosedLoopController pidController;



    public TurretSubsystem(){
        motor = new SparkMax(RobotMap.TURRET_MOTOR_ID , SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        forwardLimitSwitch = new DigitalInput(RobotMap.TURRET_FORWARD_LIMIT_SWITCH_ID);



        config.closedLoop
                .p(0)
                .i(0)
                .d(0);



        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        config.closedLoop.pid(RobotMap.TURRET_PID.kP,RobotMap.TURRET_PID.kI,RobotMap.TURRET_PID.kD);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.encoder.positionConversionFactor(1 / RobotMap.TURRET_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.TURRET_GEAR_RATIO);


        middleLimitSwitchForward = motor.getForwardLimitSwitch();
        middleLimitSwitchBackward = motor.getReverseLimitSwitch();
        encoder = motor.getEncoder();

        pidController = motor.getClosedLoopController();


    }

    public void moveToSetpoint(double angleDegrees) {
        // configure the wanted setpoint to start the PID loop
        angleDegrees = (angleDegrees/360.0) * RobotMap.TURRET_GEAR_RATIO;
        pidController.setSetpoint(angleDegrees, SparkBase.ControlType.kPosition);

    }




    public void setMotor(double speed){
        motor.set(speed);
    }

    public void stopMotor(){
        motor.stopMotor();
    }

    public boolean isForwardLimitSwitch(){
        return forwardLimitSwitch.get();
    }




    public double getPositionDegrees(){
        return encoder.getPosition() * 360;
    }
    public void setEncoder(){
        encoder.setPosition(0);
    }



    public void periodic(){
        SmartDashboard.putNumber("turretPositionDegrees: ", getPositionDegrees());
        SmartDashboard.putBoolean(("forwardLimitSwitch: ") , isForwardLimitSwitch());



    }


}
