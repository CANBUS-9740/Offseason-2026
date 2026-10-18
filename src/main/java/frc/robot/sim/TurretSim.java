package frc.robot.sim;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
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

public class TurretSim implements NTSendable {

    private static final Color8Bit MECHANISM_COLOR_SWITCH_OFF = new Color8Bit(Color.kRed);
    private static final Color8Bit MECHANISM_COLOR_SWITCH_ON = new Color8Bit(Color.kGreen);

    private final SparkMaxSim motorSim;
    private final DCMotorSim sim;
    private final SparkLimitSwitchSim forwardLimitSwitchSim;
    private final SparkLimitSwitchSim reverseLimitSwitchSim;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d mechanismCenterLine;
    private final MechanismLigament2d mechanismSwitchMin;
    private final MechanismLigament2d mechanismSwitchMax;

    public TurretSim(SparkMax motor) {
        motorSim = new SparkMaxSim(motor, RobotMap.TURRET_MOTOR);
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(RobotMap.TURRET_MOTOR, RobotMap.TURRET_MOI, RobotMap.TURRET_GEAR_RATIO), RobotMap.TURRET_MOTOR);
        forwardLimitSwitchSim = motorSim.getForwardLimitSwitchSim();
        reverseLimitSwitchSim = motorSim.getReverseLimitSwitchSim();

        mechanism = new Mechanism2d(2, 2);
        MechanismRoot2d mechanismRoot = mechanism.getRoot("base", 1, 0.8);
        mechanismCenterLine = mechanismRoot.append(new MechanismLigament2d("line", 0.9, 0, 5, new Color8Bit(Color.kRed)));
        mechanismSwitchMin = mechanism.getRoot("minRoot", 0.2, 0.5)
                .append(new MechanismLigament2d("switchBottom", 0.15, 180, 4, MECHANISM_COLOR_SWITCH_OFF));
        mechanismSwitchMax = mechanism.getRoot("maxRoot", 1.8, 0.5)
                .append(new MechanismLigament2d("switchTop", 0.15, 180, 4, MECHANISM_COLOR_SWITCH_OFF));
    }

    public void update() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double volts = motorSim.getAppliedOutput() * batteryVoltage;
        if (Math.toDegrees(sim.getAngularPositionRad()) >= RobotMap.TURRET_MAX_ANGLE && volts > 0) {
            sim.setState(Math.toRadians(RobotMap.TURRET_MAX_ANGLE), 0);
            volts = 0;
        } else if (Math.toDegrees(sim.getAngularPositionRad()) <= RobotMap.TURRET_MIN_ANGLE && volts < 0) {
            sim.setState(Math.toRadians(RobotMap.TURRET_MIN_ANGLE), 0);
            volts = 0;
        }
        sim.setInputVoltage(volts);
        sim.update(0.020);
        motorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec()),
                batteryVoltage,
                0.020);

        double posDegrees = Math.toDegrees(sim.getAngularPositionRad());
        mechanismCenterLine.setAngle(posDegrees);

        boolean atMax = posDegrees >= RobotMap.TURRET_MAX_ANGLE - 1;
        boolean atMin = posDegrees <= RobotMap.TURRET_MIN_ANGLE + 1;
        forwardLimitSwitchSim.setPressed(atMax);
        reverseLimitSwitchSim.setPressed(atMin);
        mechanismSwitchMin.setColor(atMin ? MECHANISM_COLOR_SWITCH_ON : MECHANISM_COLOR_SWITCH_OFF);
        mechanismSwitchMax.setColor(atMax ? MECHANISM_COLOR_SWITCH_ON : MECHANISM_COLOR_SWITCH_OFF);
    }

    @Override
    public void initSendable(NTSendableBuilder ntSendableBuilder) {
        mechanism.initSendable(ntSendableBuilder);
    }
}
