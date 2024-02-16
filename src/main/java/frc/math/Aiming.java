package frc.math;

import java.lang.Math;

import frc.robot.Constants.SwerveSpeedConstants;

public class Aiming {
    
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
        return Math.atan2(distance, heightDifference);
    }
    
    /**
     * 
     * @param tx The current tx value given by limelight
     * @return The output turning power
     */
    public static double getYawAdjustment(double tx){
        return tx * SwerveSpeedConstants.MaxAngularRate;
    }

}
