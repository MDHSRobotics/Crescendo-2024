package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.math.Aiming;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PoseConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private boolean m_autoRotationOverride = false;
    
    // Temporary variables for finding kSlipCurrentA. Remove after finished.
    private final TalonFX m_frontRightDriveMotor = new TalonFX(TunerConstants.kFrontRightDriveMotorId);
    private double m_voltage = 0;
    private final VoltageOut m_request = new VoltageOut(0).withEnableFOC(false);
    
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation(); // Driving forward
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation(); // Rotating robot
    // private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains(); // Rotating wheels only

    /* Routines for swerve characterization. Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    // private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Units.Volts.of(4),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(RotationCharacterization.withVolts(volts)),
    //                 null,
    //                 this));
    // private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Units.Volts.of(7),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(SteerCharacterization.withVolts(volts)),
    //                 null,
    //                 this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    /* NetworkTables logging */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StructPublisher<Pose2d> camPosepublisher = inst.getStructTopic("camPose", Pose2d.struct).publish();

    /* Shuffleboard logging */
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    private GenericEntry targetYaw = tab.add("Target Yaw", 0.0).getEntry();


    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Set the pose estimator's trust of poses from the Limelight
        setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    }
    
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // Set the pose estimator's trust of poses from the Limelight
        setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
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

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    /* The following are callbacks needed for the Path Planner Auto Builder */
    public Pose2d getPose() {
        SwerveDriveState currentState = getState();
        Pose2d currentPose = currentState.Pose;
        // System.out.println("Current pos" + currentPose.toString());

        return currentPose;
    }

    private ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveDriveState currentState = getState();
        SwerveModuleState[] moduleStates = currentState.ModuleStates;

        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
         
        //System.out.println("Getting current robot speeds " + chassisSpeeds.toString());

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

    public double getRobotYaw(){
        // System.out.println(m_odometry.getEstimatedPosition().getRotation().getDegrees());
        return getPose().getRotation().getDegrees();
    }

    /**
     * @return The new robot yaw as a Rotation2d that points the robot at the speaker.
     */
    public Rotation2d getTargetYaw() {
        Pose2d currentPose = getPose();
        Rotation2d targetYaw;

        // Calculate the yaw based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Blue) { // If blue alliance:
            targetYaw = Aiming.getYaw(PoseConstants.kBlueSpeakerPosition, currentPose);
        } else {
            targetYaw = Aiming.getYaw(PoseConstants.kRedSpeakerPosition, currentPose);
        }
        this.targetYaw.setDouble(targetYaw.getDegrees());
        return targetYaw;
    }

    public Optional<Rotation2d> getRotationTargetOverride(){
        // Some condition that should decide if we want to override rotation
        if(m_autoRotationOverride) {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(getTargetYaw());
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }


    public void setAutoRotationOverride(boolean override){
        m_autoRotationOverride = override;
    }

    // Temporary method for applying voltage to find kSlipCurrentA. Remove after finished.
    public void applyIncreasingVoltage() {
        m_voltage += 0.01;
        System.out.println(m_voltage);
        m_frontRightDriveMotor.setControl(m_request.withOutput(m_voltage));
    } 

    /* Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is disabled. */
    public void logData() {
        // Subsystem data
        double yawDegrees = getRobotYaw();
        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        double yawRateDegrees = Math.toDegrees(speeds.omegaRadiansPerSecond);

        /* Update yaw for Limelight Megatag2 */
        LimelightHelpers.SetRobotOrientation("limelight-front", yawDegrees, yawRateDegrees, 0.0, 0.0, 0.0, 0.0);
        
        /* Add Limelight Bot Pose to Pose Estimation and logs */
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        if (limelightMeasurement != null) {
            if((limelightMeasurement.tagCount >= 1) && (Math.abs(yawRateDegrees) < 720)) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                // Add camera pose to pose estimation
                addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
                // Add camera pose to SmartDashboard
                camPosepublisher.set(limelightMeasurement.pose);
                // Add camera pose to logs
                SignalLogger.writeDoubleArray("camera pose", new double[] {
                    limelightMeasurement.pose.getX(),
                    limelightMeasurement.pose.getY(),
                    limelightMeasurement.pose.getRotation().getDegrees()
                }, "", Timer.getFPGATimestamp() - limelightMeasurement.timestampSeconds);
            }
        } else {
            DriverStation.reportWarning("Could not add limelight measurement to pose estimation, make sure limelight is properly connected and configured", false);
        }
    }

}
