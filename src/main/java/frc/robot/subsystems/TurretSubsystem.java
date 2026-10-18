package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.TurretSim;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput forwardLimitSwitch;
    private final SparkLimitSwitch middleLimitSwitchForward;
    private final SparkLimitSwitch middleLimitSwitchBackward;
    private final SparkClosedLoopController pidController;

    private final TurretSim sim;

    public TurretSubsystem(){
        motor = new SparkMax(RobotMap.TURRET_MOTOR_ID , SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        forwardLimitSwitch = new DigitalInput(RobotMap.TURRET_FORWARD_LIMIT_SWITCH_ID);

        config.closedLoop
                .p(0)
                .i(0)
                .d(0);

        config.limitSwitch
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        config.closedLoop.pid(RobotMap.TURRET_PID.kP,RobotMap.TURRET_PID.kI,RobotMap.TURRET_PID.kD);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder.positionConversionFactor(1 / RobotMap.TURRET_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.TURRET_GEAR_RATIO);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        middleLimitSwitchForward = motor.getForwardLimitSwitch();
        middleLimitSwitchBackward = motor.getReverseLimitSwitch();
        encoder = motor.getEncoder();

        pidController = motor.getClosedLoopController();

        if (RobotBase.isSimulation()) {
            sim = new TurretSim(motor);
            SmartDashboard.putData("TurretSim", sim);
        } else {
            sim = null;
        }
    }

    public void moveToSetpoint(double angleDegrees) {
        // configure the wanted setpoint to start the PID loop
        double setpoint = angleDegrees / 360.0;
        pidController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
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

    public boolean isHardForwardLimitSwitch() {
        return middleLimitSwitchForward.isPressed();
    }

    public boolean isHardBackwardLimitSwitch() {
        return middleLimitSwitchBackward.isPressed();
    }

    public double getPositionDegrees(){
        return encoder.getPosition() * 360;
    }

    public boolean isAtPosition(double angleDegrees) {
        return Math.abs(getPositionDegrees() - angleDegrees) <= 1 && encoder.getVelocity() <= 5;
    }

    public void setEncoder(double angleDegrees){
        encoder.setPosition(angleDegrees);
    }

    public void periodic(){
        SmartDashboard.putNumber("turretPositionDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("forwardLimitSwitch", isForwardLimitSwitch());
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
