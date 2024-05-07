package frc.math;

import java.lang.Math;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.numbers.N3;

public class Aiming {

    private static Translation3d m_BlueSpeakerPosition = new Translation3d(0.25, 5.55, 2.0431125);

    /* Pose Estimation Aiming Methods */
    /**
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new robot yaw as a Rotation2d that points the robot at the speaker.
     */
    public static Rotation2d getYaw(Pose2d robotPose) {
        // Find the yaw of the vector
        Translation3d robotTranslation = new Translation3d(
            m_BlueSpeakerPosition.getX() - robotPose.getX(),
            m_BlueSpeakerPosition.getY() - robotPose.getY(),
            m_BlueSpeakerPosition.getZ() - ShooterConstants.kPivotHeightM); // The x, y, and z distance to the speaker
        Vector<N3> facingSpeakerVector = robotTranslation.toVector();
        Rotation3d robotRotation = new Rotation3d(facingSpeakerVector);
        Rotation2d robotYaw = new Rotation2d(robotRotation.getZ());
        return robotYaw;
    }

    /**
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new shooter pitch in degrees that points the shooter at the speaker.
     */
    public static double getPitch(Pose2d robotPose) {
        // Must find the (x,y) coordinate of the shooter's pivot point first. If you need to draw this out, go to https://www.desmos.com/calculator/1hg0vdgcf4
        double robotHeading = robotPose.getRotation().getRadians();
        double xCorrection = ShooterConstants.kPivotDistanceM * Math.cos(Math.PI + robotHeading);
        double yCorrection = ShooterConstants.kPivotDistanceM * Math.sin(Math.PI + robotHeading);

        // Find the pitch of the vector
        Translation3d robotTranslation = new Translation3d(
            m_BlueSpeakerPosition.getX() - (robotPose.getX() + xCorrection),
            m_BlueSpeakerPosition.getY() - (robotPose.getY() + yCorrection),
            m_BlueSpeakerPosition.getZ() - ShooterConstants.kPivotHeightM); // The x, y, and z distance to the speaker
        Vector<N3> facingSpeakerVector = robotTranslation.toVector();
        Rotation3d robotRotation = new Rotation3d(facingSpeakerVector);
        return Math.toDegrees(robotRotation.getY());
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
