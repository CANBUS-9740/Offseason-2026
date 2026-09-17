package frc.robot.sim;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotMap;

public class PitcherSim {

    private final SparkMaxSim pitchMotorSim;
    private final DCMotorSim pitchSim;
    private final SparkLimitSwitchSim forwardLimitSwitchSim;
    private final SparkLimitSwitchSim reverseLimitSwitchSim;

    public PitcherSim(SparkMax pitchMotor) {
        pitchMotorSim = new SparkMaxSim(pitchMotor, RobotMap.SHOOTER_PITCHER_MOTOR);
        pitchSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(RobotMap.SHOOTER_PITCHER_MOTOR, RobotMap.PITCHER_MOI, RobotMap.PITCHER_GEAR_RATIO),
                RobotMap.SHOOTER_PITCHER_MOTOR);
        forwardLimitSwitchSim = pitchMotorSim.getForwardLimitSwitchSim();
        reverseLimitSwitchSim = pitchMotorSim.getReverseLimitSwitchSim();
    }

    public void update() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double volts = pitchMotorSim.getAppliedOutput() * batteryVoltage;
        if (Math.toDegrees(pitchSim.getAngularPositionRad()) >= RobotMap.PITCHER_MAX_ANGLE_DEGREES && volts > 0) {
            pitchSim.setState(Math.toRadians(RobotMap.PITCHER_MAX_ANGLE_DEGREES), 0);
            volts = 0;
        } else if (Math.toDegrees(pitchSim.getAngularPositionRad()) <= RobotMap.PITCHER_MIN_ANGLE_DEGREES && volts < 0) {
            pitchSim.setState(Math.toRadians(RobotMap.PITCHER_MIN_ANGLE_DEGREES), 0);
            volts = 0;
        }
        pitchSim.setInputVoltage(volts);
        pitchSim.update(0.020);
        pitchMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(pitchSim.getAngularVelocityRadPerSec()),
                batteryVoltage,
                0.020);

        double posDegrees = Math.toDegrees(pitchSim.getAngularPositionRad());
        pitchMotorSim.getAbsoluteEncoderSim().setPosition(posDegrees / 360);
        forwardLimitSwitchSim.setPressed(MathUtil.isNear(RobotMap.PITCHER_MAX_ANGLE_DEGREES, posDegrees, 5));
        reverseLimitSwitchSim.setPressed(MathUtil.isNear(RobotMap.PITCHER_MIN_ANGLE_DEGREES, posDegrees, 5));
    }
}
