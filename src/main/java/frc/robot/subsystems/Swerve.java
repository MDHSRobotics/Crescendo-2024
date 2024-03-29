package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Configure AutoBuilder last
        configurePathPlanningAutoBuilder();
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Configure AutoBuilder last
        configurePathPlanningAutoBuilder();
    }

    private void configurePathPlanningAutoBuilder() {
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
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );

    }

    // Apply a request to the swerve subsystem
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));
    }

    // The following are callbacks needed for the Path Planner Auto Builder

    private Pose2d getPose() {
        SwerveDriveState currentState = getState();
        Pose2d currentPose = currentState.Pose;
        System.out.println("Current pos" + currentPose.toString());

        return currentPose;
    }

    private ChassisSpeeds getRobotRelativeSpeeds(){

        //return m_kinematics.toChassisSpeeds(getState().ModuleStates);

         SwerveDriveState currentState = getState();
         SwerveModuleState [] moduleStates = currentState.ModuleStates;
         SwerveDriveKinematics kinematics = this.m_kinematics;

         ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);
         
         System.out.println("Getting current robot speeds " + chassisSpeeds.toString());

        return chassisSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds){

        setControl(applyChassisSpeedsRequest.withSpeeds(speeds));

        System.out.println("Drive Robot Relative speeds: " + speeds.toString());
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

    public double getAngle(){
        System.out.println(m_odometry.getEstimatedPosition().getRotation().getDegrees());
        return m_odometry.getEstimatedPosition().getRotation().getDegrees();
    }
}
