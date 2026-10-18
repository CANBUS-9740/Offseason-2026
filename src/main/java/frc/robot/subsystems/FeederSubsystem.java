package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public FeederSubsystem(){

        motor = new SparkMax(RobotMap.FEEDER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder();
    }

    public void set(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.stopMotor();
    }

    public double getSpeedRPM(){
        return encoder.getVelocity();
    }


    public void periodic(){
        SmartDashboard.putNumber("FeederMotorSpeed: ", getSpeedRPM());
    }

}
