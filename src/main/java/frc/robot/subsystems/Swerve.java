package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.math.Aiming;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem, Sendable {
    private boolean m_autoRotationOverride = false;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private double m_yawRate;
    private double m_rawYawRate;


    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                    this::getPose, // Robot pose supplier
                    this::seedFieldRelative,  // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveRobotRelative, // Consumer of ChassisSpeeds to drive the robot // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                            new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(1.0, 0.0, 0), // Rotation PID constants
                            TunerConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                            driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                            new ReplanningConfig() // Default path replanning config. See the API for the options here
                    ),
                    // Boolean supplier that controls when the path will be mirrored for the red alliance.
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Reference to this subsystem to set requirements
            );
        
        // Set the method that will be used to get rotation overrides
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }


    /* The following are callbacks needed for the Path Planner Auto Builder */
    public Pose2d getPose() {
        SwerveDriveState currentState = getState();
        Pose2d currentPose = currentState.Pose;
        // System.out.println("Current pos" + currentPose.toString());

        return currentPose;
    }

    private ChassisSpeeds getRobotRelativeSpeeds(){

        //return m_kinematics.toChassisSpeeds(getState().ModuleStates);

         SwerveDriveState currentState = getState();
         SwerveModuleState [] moduleStates = currentState.ModuleStates;

         ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
         
         // System.out.println("Getting current robot speeds " + chassisSpeeds.toString());

        return chassisSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds){

        setControl(AutoRequest.withSpeeds(speeds));

        // System.out.println("Drive Robot Relative speeds: " + speeds.toString());
      }


    // Simulation
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // Apply a request to the swerve subsystem
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));
    }

    public double getAngle(){
        // System.out.println(m_odometry.getEstimatedPosition().getRotation().getDegrees());
        return m_odometry.getEstimatedPosition().getRotation().getDegrees();
    }

    public Optional<Rotation2d> getRotationTargetOverride(){
        // Some condition that should decide if we want to override rotation
        if(m_autoRotationOverride) {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(Aiming.getYaw(getPose()));
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }


    public void setAutoRotationOverride(boolean override){
        m_autoRotationOverride = override;
    }

    public void updatePose() {
        // Update yaw for Limelight Megatag2
        double yaw = getAngle();
        m_yawRate = Math.toDegrees(getRobotRelativeSpeeds().omegaRadiansPerSecond);
        m_rawYawRate = m_pigeon2.getRate(); // Megatag2 comes with its own latency compensation, so we use raw instead.
        LimelightHelpers.SetRobotOrientation("", yaw, m_rawYawRate, 0.0, 0.0, 0.0, 0.0);
        
        /* Add Limelight Bot Pose to Pose Estimation */
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        if((limelightMeasurement.tagCount >= 1) && (Math.abs(m_yawRate) < 720)) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
    }

    // Initialize the Sendable that will log values to Shuffleboard in a nice little table for us
    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("X Position", () -> getPose().getX(), null);
        builder.addDoubleProperty("Y Position", () -> getPose().getY(), null);
        builder.addDoubleProperty("X Velocity", () -> getRobotRelativeSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Y Velocity", () -> getRobotRelativeSpeeds().vyMetersPerSecond, null);
        builder.addDoubleProperty("Yaw", () -> getAngle(), null);
        builder.addDoubleProperty("Yaw Rate", () -> m_yawRate, null);
        builder.addDoubleProperty("Raw Yaw Rate", () -> m_rawYawRate, null);
    }
}
