package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameField;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTrackHubCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final GameField gameField;
    public TurretTrackHubCommand(TurretSubsystem turretSubsystem, GameField gameField){
        this.turretSubsystem = turretSubsystem;
        this.gameField = gameField;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double angle = gameField.calculateTurretAngleToHubDegrees();
        turretSubsystem.moveToSetpoint(angle);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
