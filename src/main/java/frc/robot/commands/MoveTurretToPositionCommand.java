package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretToPositionCommand extends Command {
    private final TurretSubsystem turretSubsystem;

    private double targetAngleDegrees;

    public MoveTurretToPositionCommand(TurretSubsystem turretSubsystem, double targetAngleDegrees){
        this.turretSubsystem = turretSubsystem;
        this.targetAngleDegrees = targetAngleDegrees;
        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {
        turretSubsystem.moveToSetpoint(targetAngleDegrees);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.isAtPosition(targetAngleDegrees);
    }
}
