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
        return Math.atan2(heightDifference, distance);
    }
    
    /**
     * 
     * @param tx The current tx value given by limelight
     * @return The output turning power
     */
    public static double getYawTxAdjustment(double tx){
        double value = Math.max(-SwerveSpeedConstants.MaxAngularRate, Math.min(SwerveSpeedConstants.MaxAngularRate, tx * -0.15));
        //System.out.println(value);
        return value;
    }

    public static double getYawGyroAdjustment(double yaw){
        double value = Math.max(-SwerveSpeedConstants.MaxAngularRate, Math.min(SwerveSpeedConstants.MaxAngularRate, yaw * -0.06));
        //System.out.println(value);
        return value;
    }

    public static boolean approximatelyEqual(double v1, double v2, double tolerance){
        return Math.abs(v1 - v2) < tolerance;
      }

}
