package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PitcherSystem;

public class MovePitcherToPosition extends Command {
    private final PitcherSystem pitcherSystem;
    private final double targetAngleDegrees;
    public MovePitcherToPosition(PitcherSystem pitcherSystem, double targetAngleDegrees) {
        this.pitcherSystem = pitcherSystem;
        this.targetAngleDegrees = targetAngleDegrees;
        addRequirements(pitcherSystem);

    }

    @Override
    public void initialize() {
        pitcherSystem.setPositionToPitch(targetAngleDegrees);

    }

    @Override
    public void execute() {


    }

    @Override
    public void end(boolean interrupted) {
       pitcherSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pitcherSystem.getPositionDegrees() - targetAngleDegrees) <= 1 && Math.abs(pitcherSystem.getVelocityDegrees()) <= 1;
    }
}
