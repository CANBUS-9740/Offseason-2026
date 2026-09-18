package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Optional;

public class Robot extends TimedRobot {

    private Swerve swerveSystem;
    private IntakeArmSystem intakeArmSystem;
    private IntakeCollectorSystem intakeCollectorSystem;
    private StorageSystem storageSystem;
    private TurretSubsystem turretSubsystem;
    private PitcherSystem pitcherSystem;
    private ShooterSystem shooterSystem;

    private Limelight limelight;
    private GameField gameField;
    private Pathplanner pathplanner;

    private CommandXboxController driverController;
    private CommandXboxController operationController;
    private SwerveDriveCommand swerveDriveCommand;

    private GroupCommands groupCommands;
    private SendableChooser<Command> autoChooser;


    private Command collectCommand;
    private Command stopCollectCommand;
    private boolean isCollecting = false;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        turretSubsystem = new TurretSubsystem();
        pitcherSystem = new PitcherSystem();
        shooterSystem = new ShooterSystem();

        limelight = new Limelight("limelight-forward");

        gameField = new GameField(swerveSystem);
        pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        groupCommands = new GroupCommands(swerveSystem, intakeArmSystem, intakeCollectorSystem, storageSystem, gameField);

        swerveSystem.setDefaultCommand(swerveDriveCommand);
        turretSubsystem.setDefaultCommand(new TurretTrackHubCommand(turretSubsystem, gameField));

        CommandScheduler.getInstance().onCommandInitialize((command)-> {
            System.out.printf("CMD INIT %s %s\n", command.getName(), command.getClass().getName());
        });
        CommandScheduler.getInstance().onCommandInterrupt((command, opt)-> {
            System.out.printf("CMD INT %s %s with %s\n", command.getName(), command.getClass().getName(), opt.toString());
        });
        CommandScheduler.getInstance().onCommandFinish((command)-> {
            System.out.printf("CMD FIN %s %s\n", command.getName(), command.getClass().getName());
        });

        autoChooser = new SendableChooser<>();

        // Final operation controller:

        /*collectCommand = groupCommands.intakeAndCollect();
        stopCollectCommand = groupCommands.stopIntakeAndStopCollect();

        operationController.y().onTrue(new InstantCommand(() -> {
            isCollecting = !isCollecting;

            if (isCollecting) {
                CommandScheduler.getInstance().schedule(collectCommand);
            } else {
                CommandScheduler.getInstance().schedule(stopCollectCommand);
            }
        }));


        operationController.x().onTrue(groupCommands.shootHub());
        operationController.rightBumper().onTrue(groupCommands.shootForBallTransfer());

        operationController.a().whileTrue(groupCommands.intakeUnjam1());
        operationController.start().onTrue(groupCommands.cancelAllCommands());
        operationController.b().whileTrue(groupCommands.intakeUnjam2());
        operationController.leftBumper().onTrue(groupCommands.shootForBallTransfer());
        operationController.pov(270).onTrue(groupCommands.shoot(2));

        driverController.start().onTrue(groupCommands.cancelAllCommands());



        autoChooser.addOption("dontMove", null);
        autoChooser.addOption("middle:", groupCommands.autoMiddle());
        autoChooser.addOption("right:", groupCommands.autoSideRight());
        autoChooser.addOption("left:", groupCommands.autoSideLeft());
        SmartDashboard.putData("auto chooser", autoChooser);*/
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.updateValues();
        CommandScheduler.getInstance().run();

        Optional<LimelightHelpers.PoseEstimate> poseOpt = limelight.getPose();
        if (poseOpt.isPresent()) {
            LimelightHelpers.PoseEstimate posCam = poseOpt.get();
            swerveSystem.addVisionMeasurement(posCam);
        }

        Pose2d pose2d = new Pose2d(9, 3, Rotation2d.fromDegrees(120));
        double turretAngleToHub = gameField.calculateTurretAngleToHubDegrees(pose2d);
        SmartDashboard.putNumber("TurretToHubAngle", turretAngleToHub);

        Pose2d hubPose = new Pose2d(gameField.hubPosition(), Rotation2d.kZero);
        swerveSystem.getField().getObject("Hub").setPose(hubPose);
        Pose2d turretPose = new Pose2d(pose2d.getX(), pose2d.getY(), Rotation2d.fromDegrees(turretAngleToHub + pose2d.getRotation().getDegrees()));
        swerveSystem.getField().getObject("Turret").setPose(turretPose);
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        Command auto = autoChooser.getSelected();
        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}