package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;



public class PitcherSystem extends SubsystemBase {
    private final SparkMax pitchermotor;

    private final AbsoluteEncoder encoder;
    private final SparkLimitSwitch toplimitswitch;
    private final SparkLimitSwitch bottomlimitswitch;
    private final SparkClosedLoopController pidController;





    public PitcherSystem(){
        SparkMaxConfig config; //ez
        config = new SparkMaxConfig();
        pitchermotor = new SparkMax(RobotMap.PITCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        config.absoluteEncoder //fixed mb
                .positionConversionFactor(360 / RobotMap.PITCHER_GEARBOX_RATIO) //there is a gearbox on the motor so i kept the miltiplier and changed the place
                .velocityConversionFactor(360 / RobotMap.PITCHER_GEARBOX_RATIO / 60); //same here
        config.closedLoop.pid(RobotMap.PITCHER_PID.kP,RobotMap.PITCHER_PID.kI,RobotMap.PITCHER_PID.kD).outputRange(-1,1);
        pitchermotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //moved
        pidController = pitchermotor.getClosedLoopController();
        toplimitswitch = pitchermotor.getForwardLimitSwitch();
        bottomlimitswitch = pitchermotor.getReverseLimitSwitch();
        config.limitSwitch
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        encoder = pitchermotor.getAbsoluteEncoder();

    }
    public void set(int speed){
     pitchermotor.set(speed);

    }
    public boolean isUp(){
        return toplimitswitch.isPressed();
    }
    public boolean isDown(){
        return bottomlimitswitch.isPressed(); //changed
    }
    public double getPositionDegrees(){
        return encoder.getPosition();
    }
    public double getVelocityDegrees(){
        return encoder.getVelocity();
    }
    public void setPositionToPitch(double angleDegrees){
        pidController.setSetpoint(angleDegrees, SparkMax.ControlType.kPosition);
    }
    public void stop(){
        pitchermotor.stopMotor();
    }

    @Override
    public void periodic() {
        // ITS DIAMONDS!! ITS DIAMONDS! ITS DIAMONDS!!!... no its LApeace (W speed). im keeping this.
        SmartDashboard.putNumber("PitcherSystem angleDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("PitcherSystem isUP", isUp());
        SmartDashboard.putBoolean("PitcherSystem isDOWN", isDown()); // DOWNSYNDROME REFERENCE XD
    }
}
