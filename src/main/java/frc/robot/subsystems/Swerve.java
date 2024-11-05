package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.math.Aiming;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.SwerveSpeedConstants;
import frc.robot.generated.TunerConstants;
import frc.utils.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    public enum HeadingTargets {
        SPEAKER,
        AMP_AREA,
        AMP_SHOOTING,
        STAGE_LEFT,
        STAGE_RIGHT,
        STAGE_MIDDLE,
        SOURCE
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Old PID Controller for deciding the rotational rate of the robot during speaker aiming.
    private PIDController m_rotationalRateController = new PIDController(0.15, 0, 0);
    
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation(); // Driving forward 
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation(); // Rotating robot (for HeadingController)
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
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Drive");
    private final StructPublisher<Pose2d> camPosePublisher = table.getStructTopic("camPose", Pose2d.struct).publish();
    private final StructPublisher<Rotation2d> targetYawPublisher = table.getStructTopic("Target Direction", Rotation2d.struct).publish();

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

        return currentPose;
    }

    private ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveDriveState currentState = getState();
        SwerveModuleState[] moduleStates = currentState.ModuleStates;

        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);

        return chassisSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        setControl(AutoRequest.withSpeeds(speeds));
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
        return getPose().getRotation().getDegrees();
    }

    /**
     * Finds the robot heading that points the robot at the target.
     * @param target the target you want to face
     * @param alliance your current alliance
     * @see HeadingTargets
     * @see Alliance
     */
    public Rotation2d getTargetDirection(HeadingTargets target, Alliance alliance) {
        Pose2d currentPose = getPose();
        Rotation2d targetYaw;

        // Calculate the yaw based on alliance and target
        if (alliance == Alliance.Blue) {
            switch (target) {
            case SPEAKER:
                targetYaw = Aiming.getYaw(PoseConstants.kBlueSpeaker2DPosition, currentPose);
                break;
            case AMP_AREA:
                targetYaw = Aiming.getYaw(PoseConstants.kBlueAmp2DPosition, currentPose);
                break;
            case AMP_SHOOTING:
                targetYaw = PoseConstants.facingAmp;
                break;
            case STAGE_LEFT:
                targetYaw = PoseConstants.facingBlueStageLeft;
                break;
            case STAGE_RIGHT:
                targetYaw = PoseConstants.facingBlueStageRight;
                break;
            case STAGE_MIDDLE:
                targetYaw = PoseConstants.facingBlueStageMiddle;
                break;
            default: // SOURCE:
                targetYaw = PoseConstants.behindBlueSource;
            }
        } else { // Red alliance:
            switch (target) {
            case SPEAKER:
                targetYaw = Aiming.getYaw(PoseConstants.kRedSpeaker2DPosition, currentPose);
                break;
            case AMP_AREA:
                targetYaw = Aiming.getYaw(PoseConstants.kRedAmp2DPosition, currentPose);
                break;
            case AMP_SHOOTING:
                targetYaw = PoseConstants.facingAmp;
            case STAGE_LEFT:
                targetYaw = PoseConstants.facingRedStageLeft;
                break;
            case STAGE_RIGHT:
                targetYaw = PoseConstants.facingRedStageRight;
                break;
            case STAGE_MIDDLE:
                targetYaw = PoseConstants.facingRedStageMiddle;
                break;
            default: // SOURCE:
                targetYaw = PoseConstants.behindRedSource;
            }
        }
    
        // Log the target yaw to NetworkTables
        targetYawPublisher.set(targetYaw);

        // If the alliance is red, the driveFacingAngle request will incorrectly try to rotate the target direction, so rotate it back
        if (alliance == Alliance.Red) {
            targetYaw = targetYaw.rotateBy(Rotation2d.fromDegrees(-180));
        }

        return targetYaw;
    }

    /**
     * Old method used to calculate the rotational rate based on the tx of the speaker.
     * Only works if a tag is in sight.
     * @return The rotational rate that rotates the robot to the speaker.
     */
    public double calculateTagRotationalRate() {
        double tx = LimelightHelpers.getTX("limelight-front");
        double output = m_rotationalRateController.calculate(tx, 0);
        double rotationalRate = MathUtil.clamp(output, -SwerveSpeedConstants.MaxAngularRate, SwerveSpeedConstants.MaxAngularRate);
        return rotationalRate;
    }


    /** Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is disabled. */
    public void logData() {
        /* Update yaw for Limelight Megatag2 */
        double yawDegrees = getRobotYaw();
        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        double yawRateDegrees = Math.toDegrees(speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation("limelight-front", yawDegrees, yawRateDegrees, 0.0, 0.0, 0.0, 0.0);
        
        /* Add Limelight Bot Pose to Pose Estimation and logs */
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        if (limelightMeasurement != null) {
            if((limelightMeasurement.tagCount >= 1) && (Math.abs(yawRateDegrees) < 720)) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                // Add camera pose to pose estimation
                addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
                // Add camera pose to NetworkTables
                camPosePublisher.set(limelightMeasurement.pose);
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
