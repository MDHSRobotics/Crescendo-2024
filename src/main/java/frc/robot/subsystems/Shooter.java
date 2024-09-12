package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.math.Aiming;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

  private CANSparkFlex topShooter;
  private CANSparkFlex bottomShooter;
  private CANSparkFlex angle;
  private CANSparkMax feeder;

  private SparkPIDController m_pidController;

  private RelativeEncoder m_angleEncoder;

  private double m_lastAngle = 23;

  /* Shuffleboard Logging */
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  private ComplexWidget cameraView =
    tab.addCamera("Limelight", "limelight", "10.41.41.11:5800")
      .withWidget(BuiltInWidgets.kCameraStream)
      .withSize(4, 4);

  private ShuffleboardLayout list = tab.getLayout("Shooter Info", BuiltInLayouts.kList).withSize(3, 5);
  private GenericEntry bottomShooterSpeed = list.add("Bottom Shooter Speed", 0.0).getEntry();
  private GenericEntry topShooterSpeed = list.add("Top Shooter Speed", 0.0).getEntry();
  private GenericEntry feederSpeed = list.add("Feeder Speed", 0.0).getEntry();
  private GenericEntry angleRotations = list.add("Angle Rotations", 0.0).getEntry();
  private GenericEntry angleDegrees = list.add("Angle Degrees", 0.0).getEntry();
  private GenericEntry tx = list.add("Limelight TX", 0.0).getEntry();
  private GenericEntry ty = list.add("Limelight TY", 0.0).getEntry();

  private GenericEntry shootSpeedTopAdjustment =
    tab.add("Top Shooter Speed Multiplier", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();
  private GenericEntry shootSpeedBotAdjustment =
    tab.add("Bottom Shooter Speed Multiplier", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

  private GenericEntry calculatedDistance =
    tab.add("Calculated distance", 0.0)
      .withSize(2, 1)
      .getEntry();
  private GenericEntry calculatedAngle =
    tab.add("Calculated angle", 0.0)
      .getEntry();
  private GenericEntry calculatedRotations =
    tab.add("Calculated Rotations", 0.0)
      .withSize(2, 1)
      .getEntry();
      
  private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private GenericEntry adjustment = 
    mainTab.add("Adjustment Angle", 4)
    .getEntry();
  private GenericEntry atSpeed =
    mainTab.add("At Speed", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry isAtAngle =
    mainTab.add("At Angle", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry seeTag =
    mainTab.add("Sees Tag ", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry txCorrect =
    mainTab.add("TX Correct", false)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry ready =
    mainTab.add("Ready", false)
    .withSize(4, 4)
    .getEntry();


  public Shooter(){
    topShooter = new CANSparkFlex(ShooterConstants.kTopID, MotorType.kBrushless);
    bottomShooter = new CANSparkFlex(ShooterConstants.kBottomID, MotorType.kBrushless);
    angle = new CANSparkFlex(ShooterConstants.kAngleRightID, MotorType.kBrushless);
    feeder = new CANSparkMax(ShooterConstants.kFeederID, MotorType.kBrushless);

    m_angleEncoder = angle.getEncoder();

    m_pidController = angle.getPIDController();
    m_pidController.setP(0.1);

    topShooter.setIdleMode(IdleMode.kBrake);
    bottomShooter.setIdleMode(IdleMode.kBrake);

    // Uncomment if CAN utilization is too high. Also, consider increasing the period in different ways: https://docs.revrobotics.com/brushless/spark-max/control-interfaces#use-case-examples
    // topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    // bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    // angle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
    // feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

    topShooter.setOpenLoopRampRate(0.1);
    bottomShooter.setOpenLoopRampRate(0.1);
    angle.setOpenLoopRampRate(0.1);
    feeder.setOpenLoopRampRate(0.1);
    
    //shooter2.setIdleMode(IdleMode.kCoast);
  }

  public void runShooter(double topPower, double bottomPower, double feed){
    topShooter.set(-topPower * shootSpeedTopAdjustment.getDouble(1.0));
    bottomShooter.set(bottomPower * shootSpeedBotAdjustment.getDouble(1.0));
    feeder.set(-feed);
  }

  public void rotateShooter(double angleSpeed){
    angle.set(angleSpeed);

    // Update the calculated angle so it doesn't appear to be aiming
    calculatedAngle.setDouble(0);
    calculatedRotations.setDouble(0);
  }

  //adjust the angle of the shooter
  public void setAngleFromLimelight(){
    if(tagInSight()){
      // calculate the distance
      double horizontalDistance = Aiming.calculateDistance(
          LimelightConstants.kLimelightLensHeightInches, 
          LimelightConstants.kSpeakerTagHeight,
          LimelightConstants.kLimelightMountAngleDegrees,
          ty.getDouble(0.0));
      
      // adjust the distances
      double adjustedDistance = horizontalDistance + LimelightConstants.kLimelightPivotHorizontalDistance - LimelightConstants.kSpeakerHorizontal;
      double heightDifference = LimelightConstants.kSpeakerHeight - ShooterConstants.kPivotHeight;

      // calculate the angle
      double angle = Aiming.getPitch(adjustedDistance, heightDifference);

      // Update the angle
      m_lastAngle = Math.toDegrees(Math.toDegrees(angle) + adjustment.getDouble(0));
      
      /* Logging */
      calculatedDistance.setDouble(horizontalDistance);
    }
    setAngle(m_lastAngle, true);
  }

  /** 
    * Angles the shooter to a target based on the given position.
    * @param robotPose The current robot pose given by the swerve subsystem
    */
  public void setAngleFromPose(Pose2d robotPose) {
    double targetPitch;

    // Calculate the angle based on alliance
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) { // If blue alliance:
      targetPitch = Aiming.getPitch(PoseConstants.kBlueSpeakerPosition, robotPose);
    } else {
      targetPitch = Aiming.getPitch(PoseConstants.kRedSpeakerPosition, robotPose);
    }

    // Set the angle
    setAngle(targetPitch, true);
  }

  public void setAngle(double targetAngle, boolean isAngleCalculated){
    // Calculate angle to rotations
    double rotations = ShooterConstants.kDegreesToRotationsConversion * (targetAngle - ShooterConstants.kBottomMeasureAngle);

    //Set the rotations
    if (rotations > -45.8 && rotations < 0){
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    /* Logging */
    calculatedRotations.setDouble(rotations);

    if (isAngleCalculated) {
      calculatedAngle.setDouble(targetAngle);
    } else {
      // Update the calculated values so it doesn't appear to be aiming
      calculatedAngle.setDouble(0);
      calculatedDistance.setDouble(0);
    }
  }

  public void resetEncoders(){
    angle.getEncoder().setPosition(0);
  }

  public double getAngleDegrees() {
    return angleDegrees.getDouble(0.0);
  }

  public boolean isAtAngle(){
    return Aiming.approximatelyEqual(calculatedRotations.getDouble(0), angle.getEncoder().getPosition(), 1.0);
  }

  public boolean tagInSight(){
    return (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7);
  }

  public boolean isReady(){
    return tagInSight() && atSpeed.getBoolean(false) && isAtAngle.getBoolean(false) && txCorrect.getBoolean(false);
  }

  /* Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is disabled. */
  public void logData() {
    // Listed data
    bottomShooterSpeed.setDouble(bottomShooter.get());
    topShooterSpeed.setDouble(topShooter.get());
    feederSpeed.setDouble(feeder.get());
    angleRotations.setDouble(m_angleEncoder.getPosition());
    angleDegrees.setDouble(m_angleEncoder.getPosition() / ShooterConstants.kDegreesToRotationsConversion + ShooterConstants.kBottomMeasureAngle);
    tx.setDouble(LimelightHelpers.getTX(""));
    ty.setDouble(LimelightHelpers.getTY(""));

    // Widget data
    atSpeed.setBoolean(topShooter.getEncoder().getVelocity() < -3800);
    isAtAngle.setBoolean(isAtAngle());
    seeTag.setBoolean(tagInSight());
    txCorrect.setBoolean(Aiming.approximatelyEqual(LimelightHelpers.getTX(""), 0, 2.5));
    ready.setBoolean(isReady());
  }
}
