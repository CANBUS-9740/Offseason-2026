package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
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
    private final SparkLimitSwitch middleLimitSwitch1;
    private final SparkLimitSwitch middleLimitSwitch2;
    private final SparkClosedLoopController pidController;



    public TurretSubsystem(){
        motor = new SparkMax(RobotMap.TURRET_MOTOR_ID , SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        forwardLimitSwitch = new DigitalInput(RobotMap.TURRET_FORWARD_LIMIT_SWITCH_ID);
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        config.closedLoop
                .p(0)
                .i(0)
                .d(0)
                .velocityFF(0)
                .iZone(0)
                .iMaxAccum(0) // max I accumulation ceiling
                .dFilter(0)  // filter for D component output
                .outputRange(-1, 1) // maximum output range
                .feedbackSensor(ClosedLoopConfig);


        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        middleLimitSwitch1 = motor.getForwardLimitSwitch();
        middleLimitSwitch2 = motor.getReverseLimitSwitch();


        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.closedLoop.pid(RobotMap.TURRET_PID.kP,RobotMap.TURRET_PID.kI,RobotMap.TURRET_PID.kD);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);



    }

    public void moveToSetpoint(double setPoint) {
        // configure the wanted setpoint to start the PID loop
        pidController.setReference(setPoint, SparkBase.ControlType.kPosition);

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


    public void setMoveToPosition(double positionDegrees){
        pidController.setSetpoint(positionDegrees);
    }

    public double getPositionDegrees(){
        return encoder.getPosition() * 360;
    }



    public void periodic(){
        SmartDashboard.putNumber("turretPositionDegrees", getPositionDegrees());


    }


}
