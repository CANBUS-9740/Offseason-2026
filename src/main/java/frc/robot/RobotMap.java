package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    // ----------------- Global / World constants

    public static final double GRAVITATIONAL_ACCELERATION_MPSS = 9.8;

    // ----------------- Field info

    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(104);
    public static final Pose2d RED_HUB_POSE = new Pose2d(11.862321, 4.036290, Rotation2d.kZero);

    // ----------------- Sizes / Measurements of robot characteristics

    public static final double ROBOT_LENGTH_METERS = 0.71;
    public static final double ROBOT_HEIGHT_METERS = 55;

    public static final double SHOOTER_WHEEL_MASS_KG = 0.0544;
    public static final double SHOOTER_WHEEL_MASS_KG_SMALL = 0.015876;
    public static final double SHOOTER_NEO_11_MASS_KG = 0.425;
    public static final double SHOOTER_NEO_VORTEX_MASS_KG = 0.445;
    public static final double TOTAL_SHOOTER_HEX_BORE_INCH_MASS_KG = 0.121;

    // intake arm
    public static final DCMotor INTAKE_ARM_MOTOR = DCMotor.getNEO(1);
    public static final double INTAKE_ARM_GEAR_RATIO = 1.0 / 80.0;
    public static final double INTAKE_ARM_MASS_KG = 5;
    public static final double INTAKE_ARM_LENGTH_METERS = 0.5;
    public static final double INTAKE_ARM_MOI = 1 / 3.0 * INTAKE_ARM_MASS_KG * INTAKE_ARM_LENGTH_METERS * INTAKE_ARM_LENGTH_METERS;
    public static final double INTAKE_ARM_MIN_ANGLE_DEG = 5;
    public static final double INTAKE_ARM_MAX_ANGLE_DEG = 90;
    public static final double INTAKE_ARM_START_ANGLE_RAD = INTAKE_ARM_MAX_ANGLE_DEG;
    public static final double INTAKE_ARM_MIN_ANGLE_RAD = 0;
    public static final double INTAKE_ARM_MAX_ANGLE_RAD = 92.5;

    // intake collector
    public static final DCMotor COLLECTOR_MOTOR = DCMotor.getNEO(1);
    public static final double COLLECTOR_MASS_KG = 1;
    public static final double COLLECTOR_RADIUS_M = Units.inchesToMeters(4);
    public static final double COLLECTOR_MOI = 0.5 * COLLECTOR_MASS_KG * COLLECTOR_RADIUS_M * COLLECTOR_RADIUS_M;
    public static final double COLLECTOR_GEAR_RATIO = 1;

    // storage
    public static final DCMotor STORAGE_MOTOR1 = DCMotor.getNEO(1);
    public static final double STORAGE_MASS_KG = 1;
    public static final double STORAGE_RADIUS_M = Units.inchesToMeters(4);
    public static final double STORAGE_MOI = 0.5 * STORAGE_MASS_KG * STORAGE_RADIUS_M * STORAGE_RADIUS_M;
    public static final double STORAGE_GEAR_RATIO = 1;

    // swerve
    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);

    // ----------------- Connections


    // turret
    public static final int TURRET_MOTOR_ID = 0;
    public static final int TURRET_FORWARD_LIMIT_SWITCH_ID = 0;
    public static final double TURRET_GEAR_RATIO = 15;

    public static final double FORAWRD_LIMIT_SWITCH_ANGLE = 0;
    public static final double FORAWRD_HARD_LIMIT_SWITCH_ANGLE = 90;
    public static final double BACKWARD_HARD_LIMIT_SWITCH_ANGLE = -90;



    // intake arm
    public static final int INTAKE_ARM_MOTOR_ID = 21;

    // intake collector
    public static final int COLLECTOR_MOTOR_ID = 39;

    // storage
    public static final int STORAGE_MOTOR1_ID = 20;

    // ----------------- Operational

    // collector
    public static final double COLLECTOR_SPEED =  0.8 ;//0.85;
    public static final double STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED =  0.8;

    //turret
    public static final PIDConstants TURRET_PID = new PIDConstants(0,0,0);

    // arm
    public static final double TOLERANCE_ARM_POSITION = 5;
    public static final double TOLERANCE_ARM_SPEED = 20;
    public static final double ARM_COS = 0.04;
    public static final PIDConstants ARM_PID = new PIDConstants(2.5, 0, 0);
    public static final double ARM_ENCODER_OFFSET = 0.47415367;

    //Swerve System
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.085;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PIDConstants SWERVE_PATH_DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants SWERVE_PATH_ROTATE_PID = new PIDConstants(3, 0, 0);

    //Pitcher System
    public static final int PITCHER_MOTOR_ID = 66;
    public static final double INITIAL_FIRING_ANGLE_DEGREES = 103.071 - 90;
    public static final int PITCHER_MIN_ANGLE_DEGREES = 0;
    public static final int PITCHER_MAX_ANGLE_DEGREES = 30;
    public static final PIDConstants PITCHER_PID = new PIDConstants(1,0,0);// TRIPLE UNKNOWN THIS AMAZING!
    public static final DCMotor SHOOTER_PITCHER_MOTOR = DCMotor.getNEO(1);
    public static final double PITCHER_MASS_KG = SHOOTER_WHEEL_MASS_KG*3 + SHOOTER_WHEEL_MASS_KG_SMALL*5 + SHOOTER_NEO_11_MASS_KG + SHOOTER_NEO_VORTEX_MASS_KG + TOTAL_SHOOTER_HEX_BORE_INCH_MASS_KG;
    public static final double PITCHER_LENGTH_M = 0.22;
    public static final double PITCHER_MOI = (1 / 3.0) * PITCHER_MASS_KG * PITCHER_LENGTH_M * PITCHER_LENGTH_M;
    public static final double PITCHER_GEAR_RATIO = 5;

    //Shooter System
    public static final int SHOOTER_MOTOR_ID = 67; //UNKNOWN
    public static final PIDConstants SHOOTER_PID = new PIDConstants(1,0,0); // unknown for now
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.0381;
    public static final double SHOOTER_WHEEL_RADIUS_METERS_SMALL = 0.0254;
    public static final double MAX_SHOOTER_VELOCITY_RPM = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    public static final DCMotor SHOOTER_MOTOR = DCMotor.getNeoVortex(1);
    public static final double SHOOTER_GEAR_RATIO = 1;
    public static final double SHOOTER_MOI = (1 / 2.0) * SHOOTER_WHEEL_MASS_KG * SHOOTER_WHEEL_RADIUS_METERS * SHOOTER_WHEEL_RADIUS_METERS;
    public static final Transform3d TURRET_POSE_ON_ROBOT = new Transform3d(0.370, 0,0.132, Rotation3d.kZero);
}