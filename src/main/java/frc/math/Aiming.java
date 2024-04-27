package frc.math;

import java.lang.Math;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import edu.wpi.first.math.numbers.N3;

public class Aiming {

    private static Translation3d m_BlueSpeakerPosition = new Translation3d(0.25, 5.55, 2.0431125);

    /* Pose Estimation Aiming Methods */
    private static Rotation3d getRobotRotation3d(Pose2d robotPose) {
        Translation3d robotTranslation = new Translation3d(
            m_BlueSpeakerPosition.getX() - robotPose.getX(),
            m_BlueSpeakerPosition.getY() - robotPose.getY(),
            m_BlueSpeakerPosition.getZ() - Constants.ShooterConstants.kPivotHeightM); // The x, y, and z distance to the speaker
        Vector<N3> facingSpeakerVector = robotTranslation.toVector();
        Rotation3d robotRotation = new Rotation3d(facingSpeakerVector);
        return robotRotation;
    }
    
    /**
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new robot yaw in radians that points the robot at the speaker.
     */
    public static double getYaw(Pose2d robotPose) {
        Rotation3d robotRotation = getRobotRotation3d(robotPose);
        return robotRotation.getZ();
    }

    /**
     * @param robotPose The current robot pose given by the swerve subsystem
     * @return The new robot pitch in radians that points the shooter at the speaker.
     */
    public static double getPitch(Pose2d robotPose) {
        Rotation3d robotRotation = getRobotRotation3d(robotPose);
        return robotRotation.getY();
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
