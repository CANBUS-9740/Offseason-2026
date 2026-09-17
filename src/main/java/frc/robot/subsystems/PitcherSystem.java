package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sim.PitcherSim;
import frc.robot.sim.ShooterSim;


public class PitcherSystem extends SubsystemBase {
    private final SparkMax pitchermotor;

    private final AbsoluteEncoder encoder;
    private final SparkLimitSwitch toplimitswitch;
    private final SparkLimitSwitch bottomlimitswitch;
    private final SparkClosedLoopController pidController;

    private final PitcherSim sim;

    public PitcherSystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        pitchermotor = new SparkMax(RobotMap.PITCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        config.closedLoop.pid(RobotMap.PITCHER_PID.kP, RobotMap.PITCHER_PID.kI, RobotMap.PITCHER_PID.kD)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
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

        if (RobotBase.isSimulation()) {
            sim = new PitcherSim(pitchermotor);
        } else {
            sim = null;
        }
    }

    public void set(double speed) {
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

    public double calculateFiringAngleDegrees(double distanceMeters, double firingVelocityRpm) {
        double targetHeightMeters = RobotMap.HUB_HEIGHT_METERS - RobotMap.TURRET_POSE_ON_ROBOT.getZ();
        double a = (-9.81 * distanceMeters * distanceMeters) / (2 * firingVelocityRpm * firingVelocityRpm);
        double b = distanceMeters;
        double c = a - targetHeightMeters;
        double discriminant = Math.pow(b, 2) - (4 * a * c);
        double tanRoot1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double tanRoot2 = (-b - Math.sqrt(discriminant)) / (2 * a);
        double radiant1 = Math.atan(tanRoot1);
        double radiant2 = Math.atan(tanRoot2);
        double degrees1 = Math.toDegrees(radiant1);
        double degrees2 = Math.toDegrees(radiant2);

        return Math.max(degrees1, degrees2);
    }

    @Override
    public void periodic() {
        // ITS DIAMONDS!! ITS DIAMONDS! ITS DIAMONDS!!!... no its LApeace (W speed). im keeping this.
        SmartDashboard.putNumber("PitcherSystem angleDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("PitcherSystem isUP", isUp());
        SmartDashboard.putBoolean("PitcherSystem isDOWN", isDown()); // DOWNSYNDROME REFERENCE XD
    }

    @Override
    public void simulationPeriodic() {
        sim.update();
    }
}
