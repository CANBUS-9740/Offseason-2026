package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class PitcherSystem extends SubsystemBase {
    private final SparkMax pitchermotor;

    private final AbsoluteEncoder encoder;
    private final SparkLimitSwitch toplimitswitch;
    private final SparkLimitSwitch bottomlimitswitch;
    private final SparkClosedLoopController pidController;


    public PitcherSystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        pitchermotor = new SparkMax(RobotMap.PITCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        config.closedLoop.pid(RobotMap.PITCHER_PID.kP, RobotMap.PITCHER_PID.kI, RobotMap.PITCHER_PID.kD).outputRange(-1, 1);
        config.limitSwitch
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pitchermotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = pitchermotor.getAbsoluteEncoder();
        pidController = pitchermotor.getClosedLoopController();
        toplimitswitch = pitchermotor.getForwardLimitSwitch();
        bottomlimitswitch = pitchermotor.getReverseLimitSwitch();
    }

    public void set(int speed) {
        pitchermotor.set(speed);
    }

    public boolean isUp() {
        return toplimitswitch.isPressed();
    }

    public boolean isDown() {
        return bottomlimitswitch.isPressed();
    }

    public double getPositionDegrees() {
        return encoder.getPosition() * 360;
    }

    public double getVelocityDegreesPerSecond() {
        return Math.toDegrees(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()));
    }

    public boolean isAtPosition(double angleDegrees) {
        return Math.abs(getPositionDegrees() - angleDegrees) <= 1 && Math.abs(getVelocityDegreesPerSecond()) <= 1;
    }

    public void setPositionToPitch(double angleDegrees) {
        pidController.setSetpoint(angleDegrees, SparkMax.ControlType.kPosition);
    }

    public void stop() {
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
