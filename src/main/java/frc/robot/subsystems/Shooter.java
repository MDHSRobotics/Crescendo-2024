package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.math.Aiming;
import frc.robot.LimelightHelper;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

    private CANSparkFlex topShooter;
    private CANSparkFlex bottomShooter;
    private CANSparkFlex angle1;
    private CANSparkFlex angle2;
    private CANSparkMax feeder;

    private SparkPIDController m_pidController;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry shootSpeed =
      tab.add("Max Speed", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();
    private GenericEntry angle1Rotations =
      tab.add("Left Motor Rotations", 0.0)
        .getEntry();
    private GenericEntry angle2Rotations =
      tab.add("Right Motor Rotations", 0.0)
        .getEntry();

    public Shooter(){
        topShooter = new CANSparkFlex(ShooterConstants.kTopID, MotorType.kBrushless);
        bottomShooter = new CANSparkFlex(ShooterConstants.kBottomID, MotorType.kBrushless);
        angle1 = new CANSparkFlex(ShooterConstants.kAngleLeftID, MotorType.kBrushless);
        angle2 = new CANSparkFlex(ShooterConstants.kAngleRightID, MotorType.kBrushless);
        feeder = new CANSparkMax(ShooterConstants.kFeederID, MotorType.kBrushless);

        //m_pidController = angle1.getPIDController();
        //m_pidController.setP(0.01);
        
        angle2.follow(angle1);
        SmartDashboard.putNumber("Angle 1 rotations", angle1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
        SmartDashboard.putNumber("Angle 2 rotations", angle1.getEncoder().getPosition());
        //shooter2.setIdleMode(IdleMode.kCoast);
    }

    public void runShooter(double power, double feed){
        topShooter.set(-power * shootSpeed.getDouble(1.0));
        bottomShooter.set(power * shootSpeed.getDouble(1.0));
        SmartDashboard.putNumber("Shooter Power", power);
        SmartDashboard.putNumber("Feed Power", feed);
        feeder.set(-feed);
    } 

    public void runAngle(double angle,  double feed){
        
        //System.out.println(power);
        angle1.set(angle);
        feeder.set(-feed);

        angle1Rotations.setDouble(angle1.getEncoder().getPosition());
        angle2Rotations.setDouble(angle2.getEncoder().getPosition());
    }

    //adjust the angle of the shooter
    public void setAngleFromLimelight(){
        double horizontalDistance = Aiming.calculateDistance(
            LimelightConstants.kLimelightLensHeightInches, 
            LimelightConstants.kSpeakerTagHeight,
            LimelightConstants.kLimelightMountAngleDegrees,
            LimelightHelper.getTY(""));
        
        double adjustedDistance = horizontalDistance + LimelightConstants.kLimelightPivotHorizontalDistance - LimelightConstants.kSpeakerHorizontal;
        double heightDifference = LimelightConstants.kSpeakerHeight - ShooterConstants.kPivotHeight;

        double angle = Aiming.getPitch(adjustedDistance, heightDifference);

        setAngle(angle);
    }

    public void setAngle(double angle){

        //Calculate angle to rotations


        //Set the rotations
        //m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Set Angle", angle);
    }

    public void shootSequence(){

    }

}
