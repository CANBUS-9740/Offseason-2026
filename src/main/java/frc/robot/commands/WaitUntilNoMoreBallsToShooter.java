package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class WaitUntilNoMoreBallsToShooter extends Command {
    private ShooterSystem shooterSystem;
    private double lastBallTime;


    public WaitUntilNoMoreBallsToShooter(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;

        addRequirements(shooterSystem);
    }

    public void initialize() {
        lastBallTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        if (shooterSystem.areThereBalls()) {
            lastBallTime = Timer.getFPGATimestamp();

        }
    }


    public void end(boolean interrupted) {
    }


    public boolean isFinished() {
        return Timer.getFPGATimestamp() - lastBallTime >= RobotMap.SHOOTER_TIMER_MAX_TIME;
    }


}
