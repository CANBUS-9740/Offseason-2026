package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShootAtRequiredSpeed extends Command {
    private final ShooterSystem shooterSystem;
    private final double targetspeed;
    public ShootAtRequiredSpeed(ShooterSystem shooterSystem, double targetspeed) {
        this.shooterSystem = shooterSystem;
        this.targetspeed = targetspeed;
        addRequirements(shooterSystem);

    }

    @Override
    public void initialize() {
        shooterSystem.setRotateAtVelocity(targetspeed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        shooterSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
