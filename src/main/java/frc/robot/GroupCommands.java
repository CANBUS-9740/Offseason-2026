package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCollectCommand;
import frc.robot.commands.IntakeDownCarefullyCommand;
import frc.robot.commands.IntakeTargetPositionUpCommand;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeCollectorSystem;
import frc.robot.subsystems.StorageSystem;
import frc.robot.subsystems.Swerve;

public class GroupCommands {

    private final Swerve swerveSystem;
    private final IntakeArmSystem intakeArmSystem;
    private final IntakeCollectorSystem intakeCollectorSystem;
    private final StorageSystem storageSystem;
    private final GameField gameField;

    public GroupCommands(Swerve swerveSystem,
                         IntakeArmSystem intakeArmSystem,
                         IntakeCollectorSystem intakeCollectorSystem,
                         StorageSystem storageSystem,
                         GameField gameField) {
        this.swerveSystem = swerveSystem;
        this.intakeArmSystem = intakeArmSystem;
        this.intakeCollectorSystem = intakeCollectorSystem;
        this.storageSystem = storageSystem;
        this.gameField = gameField;
    }

    public Command intakeAndCollect() {
        final var command = new ParallelCommandGroup(
                new IntakeDownCarefullyCommand(intakeArmSystem),
                new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new IntakeCollectCommand(intakeCollectorSystem)
                )
        );
        command.setName("GroupCommands.intakeAndCollect");
        return command;
    }

    public Command stopIntakeAndStopCollect() {
        final var command = new ParallelCommandGroup(
                new IntakeTargetPositionUpCommand(intakeArmSystem),
                new InstantCommand(() -> {}, intakeCollectorSystem)
        );
        command.setName("GroupCommands.stopIntakeAndStopCollect");
        return command;
    }

    public Command cancelAllCommands() {
        final var command = new InstantCommand(()-> CommandScheduler.getInstance().cancelAll());
        command.setName("GroupCommands.cancelAllCommands");
        return command;
    }
}