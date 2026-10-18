package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.sim.BallSim;
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
    private FeederSubsystem feederSubsystem;

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
    private Command waitUntilNoMoreBallsToShooterCommand;
    private BallSim ballSim;

    @Override
    public void robotInit() {
        swerveSystem = new Swerve();
        intakeArmSystem = new IntakeArmSystem();
        intakeCollectorSystem = new IntakeCollectorSystem();
        storageSystem = new StorageSystem();
        turretSubsystem = new TurretSubsystem();
        pitcherSystem = new PitcherSystem();
        shooterSystem = new ShooterSystem();
        feederSubsystem = new FeederSubsystem();

        if (RobotBase.isSimulation()) {
            ballSim = new BallSim(swerveSystem, gameField);
        } else {
            ballSim = null;
        }

        limelight = new Limelight("limelight-forward");

        gameField = new GameField(swerveSystem);
        pathplanner = new Pathplanner(swerveSystem);

        driverController = new CommandXboxController(0);
        operationController = new CommandXboxController(1);
        driverController.x().onTrue(new InstantCommand(this::launchBall));

        swerveDriveCommand = new SwerveDriveCommand(swerveSystem, driverController, false);
        groupCommands = new GroupCommands(swerveSystem, intakeArmSystem, intakeCollectorSystem, storageSystem, gameField, shooterSystem, pitcherSystem ,turretSubsystem);

        swerveSystem.setDefaultCommand(swerveDriveCommand);

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

        operationController.y().onTrue(new InstantCommand(() -> {
            isCollecting = !isCollecting;

            if (isCollecting) {
                CommandScheduler.getInstance().schedule(collectCommand);
            } else {
                CommandScheduler.getInstance().schedule(stopCollectCommand);
            }
        }));


        // Final operation controller:

        /*collectCommand = groupCommands.intakeAndCollect();
        stopCollectCommand = groupCommands.stopIntakeAndStopCollect();

        operationController.x().onTrue(groupCommands.shootHub());
        operationController.rightBumper().onTrue(groupCommands.shootForBallTransfer());
        Ultrasonic.setAutomaticMode(true);

        operationController.a().whileTrue(groupCommands.intakeUnjam1());
        operationController.start().onTrue(groupCommands.cancelAllCommands());
        operationController.b().whileTrue(groupCommands.intakeUnjam2());
        operationController.leftBumper().onTrue(groupCommands.shootForBallTransfer());
        operationController.pov(270).onTrue(groupCommands.shoot(2));
        swerveSystem.setDefaultCommand(swerveDriveCommand);
        turretSubsystem.setDefaultCommand(new TurretTrackHubCommand(turretSubsystem, gameField));

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

        Pose2d pose2d = new Pose2d(2, 3.5, Rotation2d.fromDegrees(147));
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
        ballSim.update();
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

    public void launchBall() {
        Pose3d robotPose = new Pose3d(swerveSystem.getPose());
        Pose3d shootPose = robotPose.plus(RobotMap.TURRET_POSE_ON_ROBOT);
        double firingAngleDegrees = pitcherSystem.translateSystemAngleToFiringAngle(pitcherSystem.getPositionDegrees());
        double firingDirectionDegrees = turretSubsystem.getPositionDegrees();
        double firingVelocityRpm = shooterSystem.getVelocityRPM();

        ballSim.launchBall(shootPose.getTranslation(), firingDirectionDegrees, firingVelocityRpm, firingAngleDegrees);
    }
}