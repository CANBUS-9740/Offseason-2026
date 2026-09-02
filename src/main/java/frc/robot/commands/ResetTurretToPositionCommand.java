package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ResetTurretToPositionCommand extends Command {
    private TurretSubsystem turretSubsystem;

    public ResetTurretToPositionCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        turretSubsystem.moveToSetpoint(0);
        if(turretSubsystem.isForwardLimitSwitch()){
            turretSubsystem.setEncoder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        if(turretSubsystem.getPositionDegrees() == 0){
            return true;
        }
    }
}
