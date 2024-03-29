package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.math.Aiming;
import frc.robot.LimelightHelper;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

  private CANSparkFlex topShooter;
  private CANSparkFlex bottomShooter;
  private CANSparkFlex angle;
  private CANSparkMax feeder;

  private SparkPIDController m_pidController;

  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private GenericEntry shootSpeedTop =
    tab.add("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();
  
  private GenericEntry shootSpeedBot =
    tab.add("Max Speed Bot", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();
  private GenericEntry angleRotations =
    tab.add("Angle Rotations", 0.0)
      .getEntry();
  private GenericEntry limelightTY =
    tab.add("limelight TY", 0.0)
      .getEntry();
  private GenericEntry limelightTX =
    tab.add("limelight TX", 0.0)
      .getEntry();
  private GenericEntry calculatedDistance =
    tab.add("calculated distance", 0.0)
      .getEntry();
  private GenericEntry calculatedAngle =
    tab.add("calculated angle", 0.0)
      .getEntry();
  private GenericEntry rotationsC =
    tab.add("Rotations", -0.5839)
      .getEntry();
  private GenericEntry movingAngle =
    tab.add("Moving angle motors", false)
      .getEntry();
  private GenericEntry adjustment = Shuffleboard.getTab("Main")
    .add("Adjustment Angle", 0)
    .getEntry();
  private GenericEntry atSpeed = Shuffleboard.getTab("Main")
    .add("At Speed", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry isAtAngle = Shuffleboard.getTab("Main")
    .add("At Angle", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry seeTag = Shuffleboard.getTab("Main")
    .add("Sees Tag ", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry txCorrect = Shuffleboard.getTab("Main")
    .add("TX Correct", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry ready = Shuffleboard.getTab("Main")
    .add("Ready", false)
    .withSize(4, 4)
    .getEntry();
  
  private boolean m_calibration = false;
  private boolean m_isAtAngle = false;

  public Shooter(){
    topShooter = new CANSparkFlex(ShooterConstants.kTopID, MotorType.kBrushless);
    bottomShooter = new CANSparkFlex(ShooterConstants.kBottomID, MotorType.kBrushless);
    angle = new CANSparkFlex(ShooterConstants.kAngleRightID, MotorType.kBrushless);
    feeder = new CANSparkMax(ShooterConstants.kFeederID, MotorType.kBrushless);

    m_pidController = angle.getPIDController();
    m_pidController.setP(0.1);

    topShooter.setIdleMode(IdleMode.kBrake);
    bottomShooter.setIdleMode(IdleMode.kBrake);

    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    angle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

    topShooter.setOpenLoopRampRate(0.1);
    bottomShooter.setOpenLoopRampRate(0.1);
    angle.setOpenLoopRampRate(0.1);
    feeder.setOpenLoopRampRate(0.1);
    
    SmartDashboard.putNumber("Angle 1 rotations", angle.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber("Angle 2 rotations", angle.getEncoder().getPosition());
    //shooter2.setIdleMode(IdleMode.kCoast);
  }

  public void runShooter(double topPower, double bottomPower, double feed){
    topShooter.set(-topPower * shootSpeedTop.getDouble(1.0));
    bottomShooter.set(bottomPower * shootSpeedBot.getDouble(1.0));
    SmartDashboard.putNumber("Shooter Power", bottomPower);
    SmartDashboard.putNumber("Feed Power", feed);
    feeder.set(-feed);
  } 

  public void pullBack(){
    topShooter.set(0.2);
    bottomShooter.set(-0.2);
    feeder.set(0.5);
  }

  public void run(double angleSpeed){     
    if(!m_calibration){
      setAngle(23 - 3.5);
    }else{
      angle.set(angleSpeed);
    }
    
    feeder.set(0);
    topShooter.set(0);
    bottomShooter.set(0);

    m_isAtAngle = false;
  }

  public void runFeed(double power){
    feeder.set(power);
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

    setAngle(Math.toDegrees(angle) + adjustment.getDouble(4));
    runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0);

    /* Logging */
    calculatedDistance.setDouble(horizontalDistance);
    //System.out.println((topShooter.getEncoder().getVelocity()));
    
  }

  public void setAngle(double targetAngle){
    //Calculate angle to rotations
    double rotations = rotationsC.getDouble(0.0) * (targetAngle - ShooterConstants.kBottomMeasureAngle + 4.0);

    //Set the rotations
    if(rotations > -45.8 && rotations < 0){
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      movingAngle.setBoolean(true);
    }else{
      movingAngle.setBoolean(false);
    }
    //System.out.println(rotations);

    /* Logging */
    calculatedAngle.setDouble(targetAngle + 4);
    m_isAtAngle = Aiming.approximatelyEqual(rotations, angle.getEncoder().getPosition(), 1.0);
  }

  public void resetEncoders(){
    angle.getEncoder().setPosition(0);
  }

  public void setCalibration(){
    m_calibration = !m_calibration;
  }

  public boolean isReady(){
    return tagInSight() && atSpeed.getBoolean(false) && isAtAngle.getBoolean(false) && txCorrect.getBoolean(false);
  }

  public boolean tagInSight(){
    return (LimelightHelper.getFiducialID("") == 4 || LimelightHelper.getFiducialID("") == 7);
  }

  public void logShuffleboard(){
    angleRotations.setDouble(angle.getEncoder().getPosition());
    limelightTY.setDouble(LimelightHelper.getTY(""));
    limelightTX.setDouble(LimelightHelper.getTX(""));
    atSpeed.setBoolean(topShooter.getEncoder().getVelocity() < -3800);
    isAtAngle.setBoolean(m_isAtAngle);
    seeTag.setBoolean(tagInSight());
    txCorrect.setBoolean(Aiming.approximatelyEqual(LimelightHelper.getTX(""), 0, 2.5));
  }

}
