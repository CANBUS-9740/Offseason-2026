package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    // ----------------- Global / World constants

    public static final double GRAVITATIONAL_ACCELERATION_MPSS = 9.8;

    // ----------------- Field info

    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(104);

    // ----------------- Sizes / Measurements of robot characteristics

    public static final double ROBOT_LENGTH_METERS = 0;

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
    public static final double INTAKE_ARM_MAX_ANGLE_RAD = 92.5; // last 2 lines are for compiling only
    public static final TrapezoidProfile.Constraints INTAKE_ARM_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(1000, 500);

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
    public static final double STORAGE_GENERAL_ROLLERS_BACKWARDS_LOW_SPEED = -0.2;

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
    public static final int PITCHER_MOTOR_ID = 0; //ITS unknown :(
    public static final double PITCHER_GEAR_RATIO = 0; //unknown rn
    public static final int PITCHER_TOP_LIMITSWITCH = 0; // guess what... its unknown!
    public static final int PITCHER_DOWN_LIMITSWITCH = 0; // YOU ARE NOT GONNA BELIEVE IT!!! its unknown XD...
    public static final int PITCHER_MIN_ANGLE_DEGREES = 0; // THAT ACTUALLY KNOWN
    public static final int PITCHER_MAX_ANGLE_DEGREES = 0; // DEAR INFO THIS IS UNKNOWN!!!
    public static final PIDConstants PITCHER_PID = new PIDConstants(0,0,0);// TRIPLE UNKNOWN THIS AMAZING!


}