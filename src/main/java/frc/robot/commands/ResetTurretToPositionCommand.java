package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.TurretSubsystem;

public class ResetTurretToPositionCommand extends Command {
    private TurretSubsystem turretSubsystem;

    public ResetTurretToPositionCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.setMotor(0.2);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotor();

        if (!interrupted) {
            double angle;
            if (turretSubsystem.isHardForwardLimitSwitch()) {
              angle = RobotMap.FORAWRD_HARD_LIMIT_SWITCH_ANGLE;

            } else if (turretSubsystem.isHardBackwardLimitSwitch()) {
                angle = RobotMap.BACKWARD_HARD_LIMIT_SWITCH_ANGLE;

            } else {
                angle = RobotMap.FORAWRD_LIMIT_SWITCH_ANGLE;
            }

            turretSubsystem.setEncoder(angle);
        }
    }

    @Override
    public boolean isFinished() {
        if (turretSubsystem.isHardForwardLimitSwitch() || turretSubsystem.isHardBackwardLimitSwitch()) {
            return true;
        }

        return turretSubsystem.isForwardLimitSwitch();
    }
}
