package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/**
 * This class logs swerve drive data to NetworkTables and SignalLogger at 100hz.
 *
 * <p>If you want to view the swerve drive in AdvantageScope's Swerve tab, set the max speed to
 * TunerConstants.kSpeedAt12VoltsMps, and the frame size to PathPlanner's Robot Length and Width.
 */
public class Telemetry {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Drive");
  private final StructPublisher<Pose2d> posePublisher =
      table.getStructTopic("Pose", Pose2d.struct).publish();

  /* Swerve data for AdvantageScope Swerve visualization tab */
  private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher =
      table.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> moduleTargetsPublisher =
      table.getStructArrayTopic("Module Targets", SwerveModuleState.struct).publish();
  private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher =
      table.getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();

  /* Extra data */
  private final DoublePublisher speedPublisher = table.getDoubleTopic("Robot Speed").publish();
  private final DoublePublisher odomPeriodPublisher =
      table.getDoubleTopic("Odometry Period").publish();

  /* Accept the swerve drive state and log it */
  public void telemeterize(SwerveDriveState state) {
    Pose2d pose = state.Pose;
    /* Log the pose */
    posePublisher.set(state.Pose);

    /* Log the module states and chassis speed */
    moduleStatesPublisher.set(state.ModuleStates);
    moduleTargetsPublisher.set(state.ModuleTargets);
    chassisSpeedsPublisher.set(state.speeds);

    /* Log the linear speed */
    double speed =
        Math.sqrt(
            Math.pow(state.speeds.vxMetersPerSecond, 2)
                + Math.pow(state.speeds.vyMetersPerSecond, 2));
    speedPublisher.set(speed);

    odomPeriodPublisher.set(state.OdometryPeriod);

    /* Log to SignalLogger */
    SignalLogger.writeDoubleArray(
        "odometry", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
  }
}
