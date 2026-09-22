package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeCollectorSystem;

public class WaitUntilStorageIsFull extends Command {
    private final IntakeCollectorSystem intakeCollectorSystem;

    private double ballSeenStartTime;

    public WaitUntilStorageIsFull(IntakeCollectorSystem intakeCollectorSystem) {
        this.intakeCollectorSystem = intakeCollectorSystem;

        addRequirements(intakeCollectorSystem);
    }

    @Override
    public void initialize() {
        ballSeenStartTime = 0;
    }

    @Override
    public void execute() {
        if (intakeCollectorSystem.areThereBalls()) {
            if (ballSeenStartTime == 0) {
                ballSeenStartTime = Timer.getFPGATimestamp();
            }
        } else {
            ballSeenStartTime = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return ballSeenStartTime != 0
                && Timer.getFPGATimestamp() - ballSeenStartTime >= RobotMap.STORAGE_FULL_TIME;
    }
}