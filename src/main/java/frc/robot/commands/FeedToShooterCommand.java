package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.FeederSubsystem;

public class FeedToShooterCommand extends Command {
    private final FeederSubsystem subsystem;

    public FeedToShooterCommand(FeederSubsystem subsystem){
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.set(RobotMap.FEEDER_VELOCITY_RPM);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
