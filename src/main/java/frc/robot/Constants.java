package frc.robot;


public final class Constants {
    public static final double stickDeadband = 0.1;

    public class LimelightConstants {
        // How many degrees back is your limelight rotated from perfectly vertical?
        public static final double kLimelightMountAngleDegrees = 30.0; //a1
        // The distance from the center of the Limelight lens to the floor.
        public static final double kLimelightLensHeightInches = 13.0; //w3
        // The additional horizontal distance between the limelight and the pivot
        public static final double kLimelightPivotHorizontalDistance = 1.0; //l1

        // April tag Heights
        // Additional height to account for where we measure to the top of the tag instead of the given bottom
        private static final double kTagAdditionalHeight= 7.75;
        // Height of source tags
        public static final double kSourceTagHeight = kTagAdditionalHeight + 48.125;
        // Height of speaker tags
        public static final double kSpeakerTagHeight = kTagAdditionalHeight + 51.875; //h1
        // Height of amp tags
        public static final double kAmpTagHeight = kTagAdditionalHeight + 48.125;
        // Height of stage tags
        public static final double kStageTagHeight = kTagAdditionalHeight + 47.5;

        //Height to the ideal speaker entrance in inches
        public static final double kSpeakerHeight = 80.0; // h2
        public static final double kSpeakerHorizontal = 4.0; // l2

    }

    public class ShooterConstants {

        //The max and min angles of the shooter in degrees
        public static final double kShooterMaxAngle = 90;
        public static final double kShooterMinAngle = 20;

        public static final double kPivotHeight = 1.0; //w1
    }
}