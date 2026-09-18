package frc.robot.sim;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMap;

public class PitcherSim implements NTSendable {

    private static final Color8Bit MECHANISM_COLOR_SWITCH_OFF = new Color8Bit(Color.kRed);
    private static final Color8Bit MECHANISM_COLOR_SWITCH_ON = new Color8Bit(Color.kGreen);

    private final SparkMaxSim pitchMotorSim;
    private final DCMotorSim pitchSim;
    private final SparkLimitSwitchSim forwardLimitSwitchSim;
    private final SparkLimitSwitchSim reverseLimitSwitchSim;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d mechanismPitcherLine;
    private final MechanismLigament2d mechanismSwitchBottom;
    private final MechanismLigament2d mechanismSwitchTop;

    public PitcherSim(SparkMax pitchMotor) {
        pitchMotorSim = new SparkMaxSim(pitchMotor, RobotMap.SHOOTER_PITCHER_MOTOR);
        pitchSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(RobotMap.SHOOTER_PITCHER_MOTOR, RobotMap.PITCHER_MOI, RobotMap.PITCHER_GEAR_RATIO),
                RobotMap.SHOOTER_PITCHER_MOTOR);
        forwardLimitSwitchSim = pitchMotorSim.getForwardLimitSwitchSim();
        reverseLimitSwitchSim = pitchMotorSim.getReverseLimitSwitchSim();

        mechanism = new Mechanism2d(2, 2);
        MechanismRoot2d mechanismRoot = mechanism.getRoot("base", 1.7, 0.3);
        mechanismRoot.append(new MechanismLigament2d("bottom", 1.1, 180, 2, new Color8Bit(Color.kPaleVioletRed)));
        mechanismPitcherLine = mechanismRoot.append(new MechanismLigament2d("movable", 1.1, 180, 5, new Color8Bit(Color.kRed)));
        mechanismSwitchBottom = mechanism.getRoot("bottomRoot", 0.4, 0.3)
                .append(new MechanismLigament2d("switchBottom", 0.15, 180, 4, MECHANISM_COLOR_SWITCH_OFF));
        mechanismSwitchTop = mechanism.getRoot("topRoot", 0.4, 1.3)
                .append(new MechanismLigament2d("switchTop", 0.15, 180, 4, MECHANISM_COLOR_SWITCH_OFF));
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
        mechanismPitcherLine.setAngle(180 - posDegrees);

        boolean atTop = MathUtil.isNear(RobotMap.PITCHER_MAX_ANGLE_DEGREES, posDegrees, 5);
        boolean atBottom = MathUtil.isNear(RobotMap.PITCHER_MIN_ANGLE_DEGREES, posDegrees, 5);
        forwardLimitSwitchSim.setPressed(atTop);
        reverseLimitSwitchSim.setPressed(atBottom);
        mechanismSwitchBottom.setColor(atBottom ? MECHANISM_COLOR_SWITCH_ON : MECHANISM_COLOR_SWITCH_OFF);
        mechanismSwitchTop.setColor(atTop ? MECHANISM_COLOR_SWITCH_ON : MECHANISM_COLOR_SWITCH_OFF);
    }

    @Override
    public void initSendable(NTSendableBuilder ntSendableBuilder) {
        mechanism.initSendable(ntSendableBuilder);
    }
}
