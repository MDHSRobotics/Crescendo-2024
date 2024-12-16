package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

/**
 * This command rotates the robot, and uses the gyro to calculate the average radius of the wheels.
 * This can be used in TunerConstants.kWheelRadiusInches to make our odometry more accurate. To be
 * sure that the steer motor position does not cause any inaccuracies, run this as soon as you turn
 * on the robot. All credit goes to Team 6328 Mechanical Advantage for this command.
 *
 * @see <a
 *     href="https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/264#wheel-radius-characterization-7">Mechanical
 *     Advantage's explanation</a>
 * @see <a
 *     href="https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java">Mechanical
 *     Advantage's code</a>
 */
public class WheelRadiusCharacterization extends Command {
  private final Swerve swerve;
  private final SwerveRequest.ApplyChassisSpeeds rotationRequest =
      new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveRequest.ApplyChassisSpeeds stopRequest =
      new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds());

  // Max rotation speed in radians per second.
  private final double maxRotationalRate = 1;
  // This limits the rotational acceleration to 1 radians per second^2 to prevent any wheel slip
  // when the robot starts rotating.
  // If the acceleration is the same as the rotational rate, that means it will take 1 second to get
  // to the max rotational rate.
  private final SlewRateLimiter rotationalAccelerationLimiter = new SlewRateLimiter(1);

  private double lastGyroYawRadians = 0.0;
  private double totalGyroYawRadians = 0.0;

  // Wheel positions in radians.
  private double[] startingWheelPositions;

  // Calculated wheel radius in inches.
  private double currentEffectiveWheelRadius = 0.0;

  /* NetworkTables logging */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table =
      inst.getTable("Drive").getSubTable("Wheel Radius Characterization");
  private final DoublePublisher averageWheelPositionPublisher =
      table.getDoubleTopic("Average Wheel Position").publish();
  private final DoublePublisher totalYawPublisher =
      table.getDoubleTopic("Total Yaw Radians").publish();
  private final DoublePublisher wheelRadiusPublisher =
      table.getDoubleTopic("Current Wheel Radius Inches").publish();

  public WheelRadiusCharacterization(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRadians = swerve.getRobotYawRadians();
    totalGyroYawRadians = 0.0;

    startingWheelPositions = swerve.getWheelPositions();

    rotationalAccelerationLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Rotate the swerve drive
    double rotationalRate = rotationalAccelerationLimiter.calculate(maxRotationalRate);
    rotationRequest.Speeds.omegaRadiansPerSecond = rotationalRate;
    swerve.setControl(rotationRequest);

    // Get yaw and wheel positions
    double currentGyroYawRadians = swerve.getRobotYawRadians();
    totalGyroYawRadians += MathUtil.angleModulus(currentGyroYawRadians - lastGyroYawRadians);
    lastGyroYawRadians = currentGyroYawRadians;

    double averageWheelPosition = 0.0;
    double[] wheelPositions = swerve.getWheelPositions();
    for (int i = 0; i < 4; ++i) {
      averageWheelPosition += Math.abs(wheelPositions[i] - startingWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    double currentEffectiveWheelRadiusMeters =
        (totalGyroYawRadians * TunerConstants.kDriveBaseRadius) / averageWheelPosition;
    currentEffectiveWheelRadius = Units.metersToInches(currentEffectiveWheelRadiusMeters);

    // Log values to NetworkTables
    averageWheelPositionPublisher.set(averageWheelPosition);
    totalYawPublisher.set(totalGyroYawRadians);
    wheelRadiusPublisher.set(currentEffectiveWheelRadius);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(stopRequest);
    if (totalGyroYawRadians <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println("Effective Wheel Radius: " + currentEffectiveWheelRadius + " inches");
    }
  }
}
