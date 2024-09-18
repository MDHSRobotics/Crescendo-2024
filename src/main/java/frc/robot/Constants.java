package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public class SwerveSpeedConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        // https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/3
        // https://study.com/skill/learn/converting-angular-speed-from-revolutions-per-second-to-radians-per-second-explanation.html#:~:text=Steps%20for%20Converting%20Angular%20Speed,r%20a%20d%201%20revolution%20.
        public static final double MaxAngularRate = 11.89;
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

    public class PoseConstants {
        // Initial vector of the robot orientation, representing a robot with no rotation.
        public static final Vector<N3> facingForwardVector = VecBuilder.fill(1, 0, 0);

        // 3D position of each speaker opening in meters
        public static final Translation3d kBlueSpeakerPosition = new Translation3d(0.25, 5.55, 2.0431125);
        public static final Translation3d kRedSpeakerPosition = new Translation3d(16.3, 5.55, 2.0431125);
    }

    public class ShooterConstants {

        /* Motor IDs */
        public static final int kTopID = 3;
        public static final int kBottomID = 4;
        public static final int kAngleLeftID = 5;
        public static final int kAngleRightID = 6;
        public static final int kFeederID = 7;

        // Limit Switch ID
        public static final int kLimitSwitchID = 0;

        //The max and min angles of the shooter in degrees
        public static final double kShooterMaxAngle = 90;
        public static final double kShooterMinAngle = 23;

        // Height of the shooter from the ground in inches
        public static final double kPivotHeight = 12.375; //w1
        // Height of the shooter from the ground in meters
        public static final double kPivotHeightM = 0.314325;
        // Horizontal distance of the pivot from the center of the robot in meters
        public static final double kPivotDistanceM = -0.18415;

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

        // Limit Switch ID
        public static final int kLimitSwitchID = 2;
    }

    public class LEDConstants {

        public static final int kStripLength = 35;

        public static final int kStrobeLength = 40;
    }
}

/*
CAN Sequence
-RoboRIO
- Shooter Top Motor - kTopID
-Shooter Bottom Motor - kBottomID

 */