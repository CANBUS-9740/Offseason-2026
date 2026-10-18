package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Set;

public class GroupCommands {

    private final Swerve swerveSystem;
    private final IntakeArmSystem intakeArmSystem;
    private final IntakeCollectorSystem intakeCollectorSystem;
    private final StorageSystem storageSystem;
    private final GameField gameField;
    private final ShooterSystem shooterSystem;
    private final PitcherSystem pitcherSystem;
    private final TurretSubsystem turretSubsystem;

    public GroupCommands(Swerve swerveSystem,
                         IntakeArmSystem intakeArmSystem,
                         IntakeCollectorSystem intakeCollectorSystem,
                         StorageSystem storageSystem,
                         GameField gameField,
                         ShooterSystem shooterSystem,
                         PitcherSystem pitcherSystem,
                         TurretSubsystem turretSubsystem) {
        this.swerveSystem = swerveSystem;
        this.intakeArmSystem = intakeArmSystem;
        this.intakeCollectorSystem = intakeCollectorSystem;
        this.storageSystem = storageSystem;
        this.gameField = gameField;
        this.shooterSystem = shooterSystem;
        this.pitcherSystem = pitcherSystem;
        this.turretSubsystem = turretSubsystem;
    }
    public Command shootToHub() {
        Command command = Commands.defer(()-> {
            Pose2d robotPose = swerveSystem.getPose();
            double angleForTurret = 0;
            double distance = gameField.getDistanceToHub(robotPose);
            double velocity = 0;
            double firingAngle = pitcherSystem.calculateFiringAngleDegrees(distance, velocity);
            return new ParallelCommandGroup(
                    new MoveTurretToPositionCommand(turretSubsystem, angleForTurret),
                    new ShootAtRequiredSpeed(shooterSystem, velocity),
                    new MovePitcherToPosition(pitcherSystem, firingAngle)
                    // new FeedToShooterCommand(storageSystem)
            );
        }, Set.of(shooterSystem, pitcherSystem/*, feederSystem*/, turretSubsystem));

        command.setName("GroupCommands.shootToHub");
        return command;
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