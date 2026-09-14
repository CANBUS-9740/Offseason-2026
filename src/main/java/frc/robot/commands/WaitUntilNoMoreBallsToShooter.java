package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class WaitUntilNoMoreBallsToShooter extends Command {
    private ShooterSystem shooterSystem;
    private Ultrasonic ultrasonic;
    private double currentTime;

    public WaitUntilNoMoreBallsToShooter(ShooterSystem shooterSystem, Ultrasonic ultrasonic){
        this.shooterSystem = shooterSystem;
        this.ultrasonic = ultrasonic;

        addRequirements(shooterSystem);
    }
    public void initialize() {
    }

    public void execute() {
        currentTime = Timer.getFPGATimestamp();

    }


    public void end(boolean interrupted) {}


    public boolean isFinished() {
        if(currentTime == RobotMap.SHOOTER_TIMER_MAX_TIME){
        }
        return true;
    }


}
