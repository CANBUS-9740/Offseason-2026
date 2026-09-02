package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretToPositionCommand extends Command {
    private final TurretSubsystem turretSubsystem;

    private double target;

    private double speed;


    public MoveTurretToPositionCommand(TurretSubsystem turretSubsystem,double target,double speed){
        this.turretSubsystem = turretSubsystem;

        this.target = target;
        this.speed = speed;


        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {
        turretSubsystem.moveToSetpoint(target);
    }

    @Override
    public void execute() {
        turretSubsystem.setMotor(speed);

    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.getPositionDegrees() == target;
    }
}
