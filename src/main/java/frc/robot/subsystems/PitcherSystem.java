package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.lang.module.Configuration;

public class PitcherSystem extends SubsystemBase {
    private final SparkMax pitchermotor;
    private final SparkMaxConfig config;
    private final AbsoluteEncoder encoder;
    private final SparkLimitSwitch toplimitswitch;
    private final SparkLimitSwitch bottomlimitswitch;
    private final SparkClosedLoopController pidController;





    public PitcherSystem(){
        config = new SparkMaxConfig();
        pitchermotor = new SparkMax(RobotMap.PITCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        pidController = pitchermotor.getClosedLoopController(); // im not sure if right
        toplimitswitch = pitchermotor.getForwardLimitSwitch();
        bottomlimitswitch = pitchermotor.getReverseLimitSwitch();
        config.limitSwitch
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        encoder = pitchermotor.getAbsoluteEncoder();
        config.encoder
                .positionConversionFactor(360 / RobotMap.PITCHER_GEAR_RATIO) //dont know what to put here. update: found what to put here
                .velocityConversionFactor(360 / RobotMap.PITCHER_GEAR_RATIO / 60); //same here
        pitchermotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.closedLoop.pid(RobotMap.PITCHER_PID.kP,RobotMap.PITCHER_PID.kI,RobotMap.PITCHER_PID.kD).outputRange(-1,1);
    }
    public void set(int speed){
     pitchermotor.set(speed);

    }
    public boolean isUp(){
        boolean isTopPressed = toplimitswitch.isPressed();
        return isTopPressed;
    }
    public boolean isDown(){
        boolean isBottomPressed = bottomlimitswitch.isPressed();
        return isBottomPressed;
    }
    public double getPositionDegrees(){
        return encoder.getPosition();
    }
    public void setPositionToPitch(double angleDegrees){
        pidController.setSetpoint(angleDegrees, SparkMax.ControlType.kPosition);
    }
    public void stop(){
        pitchermotor.stopMotor();
    }

    @Override
    public void periodic() {
        // ITS DIAMONDS!! ITS DIAMONDS! ITS DIAMONDS!!!... no its LApeace (W speed).
        SmartDashboard.putNumber("PitcherSystem angleDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("PitcherSystem isUP", isUp());
        SmartDashboard.putBoolean("PitcherSystem isDOWN", isDown()); // DOWNSYNDROME REFERENCE XD
    }
}
