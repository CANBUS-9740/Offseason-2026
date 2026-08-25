package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.lang.module.Configuration;

public class PitcherSystem extends SubsystemBase {
    private final SparkMax pitchermotor;
    private final SparkMaxConfig config;
    private final AbsoluteEncoder encoder;
    private final DigitalInput toplimitswitch;
    private final DigitalInput bottomlimitswitch;




    public PitcherSystem(){
        config = new SparkMaxConfig();
        pitchermotor = new SparkMax(RobotMap.PITCHER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        toplimitswitch = new DigitalInput(RobotMap.PITCHER_TOP_LIMITSWITCH);
        bottomlimitswitch = new DigitalInput(RobotMap.PITCHER_DOWN_LIMITSWITCH);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        encoder = pitchermotor.getAbsoluteEncoder();
        config.encoder
                .positionConversionFactor(0)
                .velocityConversionFactor(0);
        pitchermotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        config.closedLoop.pid(RobotMap.PITCHER_PID.kP,RobotMap.PITCHER_PID.kI,RobotMap.PITCHER_PID.kD);
    }
    public void set(int speed){
     pitchermotor.set(speed);

    }
    public boolean isUp(){
        return toplimitswitch.get();
    }
    public boolean isDown(){
        return bottomlimitswitch.get();
    }
    public double getPositionDegrees(){
        return encoder.getPosition();
    }
    public void setPositionToPitch(double angleDegrees){
        angleDegrees = getPositionDegrees(); // i know i need to multiply by something but dont know what or the equestion
        // also i dont know what you wanted in "move motor with PID of SparkMax"
    }
    public void stop(){
        pitchermotor.stopMotor();
    }

    @Override
    public void periodic() {
        // ITS DIAMONDS!! ITS DIAMONDS! ITS DIAMONDS!!!... no its LApeace (W speed).
        SmartDashboard.putNumber("PitcherSystem angleDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("PitcherSystem isUP", isUp());
        SmartDashboard.putBoolean("PitcherSystem isDOWN", isDown()); // DOWNSYNDROME REFERENCE XD
    }
}
