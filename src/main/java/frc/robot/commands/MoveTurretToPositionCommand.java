package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretToPositionCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private double positionDegrees;
    private double target;
    private PIDController pidController;
    private double speed;


    public MoveTurretToPositionCommand(TurretSubsystem turretSubsystem,double positionDegrees,double target,double speed){
        this.turretSubsystem = turretSubsystem;
        this.positionDegrees = positionDegrees;
        this.target = target;
        this.speed = speed;

        pidController = new PIDController(0,0,0);
        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {
        turretSubsystem.setMoveToPosition(target);
    }

    @Override
    public void execute() {
        speed = pidController.calculate(positionDegrees,target);
        turretSubsystem.setMotor(speed);


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
