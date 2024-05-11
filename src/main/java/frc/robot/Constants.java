package frc.robot;

import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public class SwerveSpeedConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    }

    public class LimelightConstants {
        // How many degrees back is your limelight rotated from perfectly vertical?
        public static final double kLimelightMountAngleDegrees = 30.0; //a1
        // The distance from the center of the Limelight lens to the floor.
        public static final double kLimelightLensHeightInches = 10.992; //w3
        // The additional horizontal distance between the limelight and the pivot
        public static final double kLimelightPivotHorizontalDistance = 19.446; //l1

        // April tag Heights
        // Additional height to account for where we measure to the top of the tag instead of the given bottom
        private static final double kTagAdditionalHeight = 7.75;
        // Height of source tags
        public static final double kSourceTagHeight = kTagAdditionalHeight + 48.125;
        // Height of speaker tags
        public static final double kSpeakerTagHeight = kTagAdditionalHeight + 51.875; //h1
        // Height of amp tags
        public static final double kAmpTagHeight = kTagAdditionalHeight + 48.125;
        // Height of stage tags
        public static final double kStageTagHeight = kTagAdditionalHeight + 47.5;

        //Height to the ideal speaker entrance in inches
        public static final double kSpeakerHeight = 86.0; // h2
        public static final double kSpeakerHorizontal = 9.0; // l2

    }

    public class ShooterConstants {

        /* Motor IDs */
        public static final int kTopID = 3;
        public static final int kBottomID = 4;
        public static final int kAngleLeftID = 5;
        public static final int kAngleRightID = 6;
        public static final int kFeederID = 7;

        //The max and min angles of the shooter in degrees
        public static final double kShooterMaxAngle = 90;
        public static final double kShooterMinAngle = 20;

        // Height of the shooter from the ground in inches
        public static final double kPivotHeight = 12.375; //w1
        // Height of the shooter from the ground in meters
        public static final double kPivotHeightM = 0.314325;
        // Horizontal distance of the pivot from the center of the robot in meters
        public static final double kPivotDistanceM = 0.18415;

        // Gear ratio between the pivot and motors
        public static final double kAngleGearRatio = 1920;

        // At some angle, measure the angle and rotations and the code will account for it when calculating the rotations
        public static final double kBottomMeasureAngle = 23.0;
        public static final double kDegreesToRotationsConversion = -0.5839;

        public static final double ampAngle = 50.0;

        public static final double ampTopSpeed = 0.175;
        public static final double ampBottomSpeed = 0.525;
        public static final double speakerSpeed = 0.7;
    }

    public class IntakeConstants {

        /* Motor IDs */
        public static final int kLeftAngleID = 8;
        public static final int kRightAngleID = 9;
        public static final int kConveyorID = 10;
        public static final int kIntakeID = 11;
    }

    public class ClimbConstants {
        
        /* Motor IDs */
        public static final int kLeftClimbMotorID = 1;
        public static final int kRightClimbMotorID = 2;

        public static final int leftTopLimit = 0;
        public static final int rightTopLimit = 0;
        public static final int leftBottomLimit = 10;
        public static final int rightBottomLimit = 10;

    }

    public class LEDConstants {

        public static final int kStripLength = 35;

        public static final int kStrobeLength = 40;
    }
}