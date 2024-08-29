package frc.math;

import java.lang.Math;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.numbers.N3;

public class Aiming {

    /* Pose Estimation Aiming Methods */
    /**
     * @param targetPose The 3D position of the target (found in PoseConstants)
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new robot yaw as a Rotation2d that points the robot at the speaker.
     */
    public static Rotation2d getYaw(Translation3d goalPose, Pose2d robotPose) {
        // Find the yaw of the vector between the two points
        Translation3d robotTranslation = new Translation3d(
            goalPose.getX() - robotPose.getX(),
            goalPose.getY() - robotPose.getY(),
            goalPose.getZ() - ShooterConstants.kPivotHeightM); // The x, y, and z distance to the speaker
        Vector<N3> facingSpeakerVector = robotTranslation.toVector();
        Rotation3d robotRotation = new Rotation3d(PoseConstants.facingForwardVector, facingSpeakerVector); // Rotation from forward to facing the speaker.
        Rotation2d robotYaw = new Rotation2d(robotRotation.getZ());

        return robotYaw;
    }

    /**
     * @param targetPose The 3D position of the target, given by the swerve subsystem
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new shooter pitch in degrees that points the shooter at the speaker.
     */
    public static double getPitch(Translation3d goalPose, Pose2d robotPose) {
        // Must find the (x,y) coordinate of the shooter's pivot point first. If you need to draw this out to understand, go to https://www.desmos.com/calculator/1hg0vdgcf4
        double robotHeading = robotPose.getRotation().getRadians();
        double xCorrection = ShooterConstants.kPivotDistanceM * Math.cos(Math.PI + robotHeading);
        double yCorrection = ShooterConstants.kPivotDistanceM * Math.sin(Math.PI + robotHeading);

        // Find the pitch of the vector
        Translation3d robotTranslation = new Translation3d(
            goalPose.getX() - (robotPose.getX() + xCorrection),
            goalPose.getY() - (robotPose.getY() + yCorrection),
            goalPose.getZ() - ShooterConstants.kPivotHeightM); // The x, y, and z distance to the speaker
        Vector<N3> facingSpeakerVector = robotTranslation.toVector();
        Rotation3d robotRotation = new Rotation3d(PoseConstants.facingForwardVector, facingSpeakerVector); // Rotation from forward to facing the speaker.
        
        // We take the negative because we want the clockwise angle.
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system
        return -Math.toDegrees(robotRotation.getY());
    }

    /* Limelight Aiming Methods */
    /**
     * @param lensHeight The height of the limelight in inches
     * @param goalHeight The height of the goal in inches
     * @param initialAngle The mounting angle of the limelight in degrees
     * @param offsetAngle The additional angle to the target in degrees
     * @return Distance from the limelight to the apriltag in inches
     */
    public static double calculateDistance(double lensHeight, double goalHeight, double initialAngle, double offsetAngle){
        double angleToGoalRadians = Math.toRadians(initialAngle + offsetAngle);

        //Calculate distance
        return (goalHeight - lensHeight) / Math.tan(angleToGoalRadians);
    }

    /**
     * @param distance Distance from the limelight to the apriltag in inches
     * @param heightDifference Distance to the height of the apriltag in inches
     * @return Target pitch angle in radians 
     */
    public static double getPitch(double distance, double heightDifference){
        return Math.atan2(heightDifference, distance);
    }

    public static boolean approximatelyEqual(double v1, double v2, double tolerance){
        return Math.abs(v1 - v2) < tolerance;
      }

}
