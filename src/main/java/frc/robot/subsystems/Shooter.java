package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.math.Aiming;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

    private CANSparkFlex topShooter;
    private CANSparkFlex bottomShooter;
    private CANSparkFlex angle;
    private CANSparkMax feeder;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_angleEncoder;

    private double m_lastAngle = 23;

    /* Shuffleboard Logging */
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private ShuffleboardLayout list =
            tab.getLayout("Shooter Info", BuiltInLayouts.kList).withSize(3, 5);
    private GenericEntry bottomShooterSpeed =
            list.add("Bottom Shooter Speed", 0.0).getEntry();
    private GenericEntry topShooterSpeed = list.add("Top Shooter Speed", 0.0).getEntry();
    private GenericEntry feederSpeed = list.add("Feeder Speed", 0.0).getEntry();
    private GenericEntry angleRotations = list.add("Angle Rotations", 0.0).getEntry();
    private GenericEntry angleDegrees = list.add("Angle Degrees", 0.0).getEntry();
    private GenericEntry tx = list.add("Limelight TX", 0.0).getEntry();
    private GenericEntry ty = list.add("Limelight TY", 0.0).getEntry();

    private GenericEntry calculatedDistance =
            tab.add("Calculated distance", 0.0).withSize(2, 1).getEntry();
    private GenericEntry calculatedAngle =
            tab.add("Calculated angle", 0.0).withSize(2, 1).getEntry();
    private GenericEntry calculatedRotations =
            tab.add("Calculated Rotations", 0.0).withSize(3, 1).getEntry();

    private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private GenericEntry adjustment =
            mainTab.addPersistent("Adjustment Angle", 3).withSize(2, 1).getEntry();
    private GenericEntry atSpeed = mainTab.add("At Speed", false).withSize(2, 1).getEntry();
    private GenericEntry isAtAngle =
            mainTab.add("At Angle", false).withSize(1, 1).getEntry();
    private GenericEntry seeTag = mainTab.add("Sees Tag ", false).withSize(2, 1).getEntry();
    private GenericEntry txCorrect =
            mainTab.add("TX Correct", false).withSize(2, 1).getEntry();
    private GenericEntry ready = mainTab.add("Ready", false).withSize(2, 2).getEntry();

    public Shooter() {
        topShooter = new CANSparkFlex(ShooterConstants.kTopID, MotorType.kBrushless);
        bottomShooter = new CANSparkFlex(ShooterConstants.kBottomID, MotorType.kBrushless);
        angle = new CANSparkFlex(ShooterConstants.kAngleID, MotorType.kBrushless);
        feeder = new CANSparkMax(ShooterConstants.kFeederID, MotorType.kBrushless);

        topShooter.setInverted(true);

        m_angleEncoder = angle.getEncoder();

        m_pidController = angle.getPIDController();
        m_pidController.setP(0.1);

        topShooter.setIdleMode(IdleMode.kBrake);
        bottomShooter.setIdleMode(IdleMode.kBrake);

        // CAN optimization:
        // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
        topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        angle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        angle.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        feeder.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);

        topShooter.setOpenLoopRampRate(0.1);
        bottomShooter.setOpenLoopRampRate(0.1);
        angle.setOpenLoopRampRate(0.1);
        feeder.setOpenLoopRampRate(0.1);
    }

    // adjust the angle of the shooter
    public void setAngleFromLimelight() {
        if (tagInSight()) {
            // calculate the distance
            double horizontalDistance = Aiming.calculateDistance(
                    LimelightConstants.kLimelightLensHeightInches,
                    LimelightConstants.kSpeakerTagHeight,
                    LimelightConstants.kLimelightMountAngleDegrees,
                    ty.getDouble(0.0));

            // adjust the distances
            double adjustedDistance = horizontalDistance
                    + LimelightConstants.kLimelightPivotHorizontalDistance
                    - LimelightConstants.kSpeakerHorizontal;
            double heightDifference = LimelightConstants.kSpeakerHeight - ShooterConstants.kPivotHeight;

            // calculate the angle
            double angle = Aiming.getPitch(adjustedDistance, heightDifference);

            // Update the angle
            m_lastAngle = Math.toDegrees(angle) + adjustment.getDouble(0);

            /* Logging */
            calculatedDistance.setDouble(horizontalDistance);
        }
        setAngle(m_lastAngle, true);
    }

    /**
     * Angles the shooter to a target based on the given position.
     *
     * @param robotPose The current robot pose given by the swerve subsystem
     */
    public void setAngleFromPose(Pose2d robotPose) {
        double targetPitch;
        Alliance alliance = DriverStation.getAlliance().orElseThrow();

        // Calculate the angle based on alliance
        if (alliance == Alliance.Blue) { // If blue alliance:
            targetPitch = Aiming.getPitch(PoseConstants.kBlueSpeaker3DPosition, robotPose);
        } else {
            targetPitch = Aiming.getPitch(PoseConstants.kRedSpeaker3DPosition, robotPose);
        }

        // Set the angle
        setAngle(targetPitch, true);
    }

    public void setAngle(double targetAngle, boolean isAngleCalculated) {
        // Calculate angle to rotations
        double rotations =
                ShooterConstants.kDegreesToRotationsConversion * (targetAngle - ShooterConstants.kBottomMeasureAngle);

        // Set the rotations
        if (rotations > -45.8 && rotations <= 0) {
            m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        }

        /* Logging */
        calculatedRotations.setDouble(rotations);

        if (isAngleCalculated) {
            calculatedAngle.setDouble(targetAngle);
        } else {
            // Update the calculated values to 0 so it doesn't appear to be aiming
            calculatedAngle.setDouble(0);
            calculatedDistance.setDouble(0);
        }
    }

    public boolean isAtAngle() {
        return Aiming.approximatelyEqual(
                calculatedRotations.getDouble(0), angle.getEncoder().getPosition(), 1.0);
    }

    public boolean tagInSight() {
        Alliance alliance = DriverStation.getAlliance().orElseThrow();
        double tagID = LimelightHelpers.getFiducialID("limelight-front");
        if (alliance == Alliance.Blue && tagID == 7) {
            return true;
        } else if (alliance == Alliance.Red && tagID == 4) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isReady() {
        return tagInSight() && atSpeed.getBoolean(false) && isAtAngle.getBoolean(false) && txCorrect.getBoolean(false);
    }

    /* Instance Command Factory Methods
     * These methods allow us to create single-subsystem commands directly in the subsystems, instead of placing them in RobotContainer.
     * https://docs.wpilib.org/en/latest/docs/software/commandbased/organizing-command-based.html#instance-command-factory-methods
     */

    /**
     * This command turns off the flywheels and rollers, and lowers the shooter to its bottom
     * position.
     */
    public Command disableShooterCommand() {
        return this.runOnce(() -> {
                    topShooter.set(0);
                    bottomShooter.set(0);
                    feeder.set(0);
                    setAngle(ShooterConstants.kBottomMeasureAngle, false);
                })
                .andThen(Commands.idle(this))
                .withName("Disable Shooter");
    }

    /**
     * This command lowers the shooter to its bottom position and runs the feeder.
     *
     * <p>The shooter must lower and run the feeder, or else the note will get stuck in the robot.
     */
    public Command intakeNoteCommand() {
        return this.runOnce(() -> {
                    topShooter.set(0);
                    bottomShooter.set(0);
                    feeder.set(0.6);
                    setAngle(ShooterConstants.kBottomMeasureAngle, false);
                })
                .andThen(Commands.idle(this));
    }

    /**
     * This command runs the shooter in reverse so the note is away from the flywheel, and then spins
     * up the flywheels. The shooter must run in reverse first, or else the note will fly out when the
     * flywheels spin up.
     *
     * <p>This is meant to be used in other commands.
     */
    public Command spinUpFlywheelsCommand() {
        return this.startEnd(
                        () -> {
                            topShooter.set(-0.2);
                            bottomShooter.set(-0.2);
                            feeder.set(-0.5);
                        },
                        () -> {
                            topShooter.set(ShooterConstants.speakerSpeed);
                            bottomShooter.set(ShooterConstants.speakerSpeed);
                            feeder.set(0);
                        })
                .withTimeout(0.1);
    }

    /**
     * This command spins up the flywheels, and then it constantly calculates the angle to the speaker
     * using the limelight.
     */
    public Command aimShooterCommand() {
        return Commands.sequence(spinUpFlywheelsCommand(), this.run(() -> {
                    setAngleFromLimelight();
                }))
                .withName("Aim Shooter using Limelight");
    }

    /**
     * This command spins up the flywheels, and then it constantly calculates the angle to the speaker
     * using the supplied robot position.
     *
     * <p>You must pass a pose supplier method into this command instead of a single pose, because the
     * command needs to get the pose every update.
     *
     * @param poseSupplier A method that returns the robot position, ideally s_Swerve.getPose().
     */
    public Command aimShooterWithPoseCommand(Supplier<Pose2d> poseSupplier) {
        return Commands.sequence(spinUpFlywheelsCommand(), this.run(() -> {
                    setAngleFromPose(poseSupplier.get());
                }))
                .withName("Aim Shooter using Pose");
    }

    /**
     * This command spins up the flywheels, and then it sets the shooter to the specified angle.
     *
     * @param angle The angle from the horizontal (the floor) to the shooter.
     */
    public Command aimShooterWithAngleCommand(double angle) {
        return Commands.sequence(
                        spinUpFlywheelsCommand(),
                        this.runOnce(() -> {
                            setAngle(angle, false);
                        }),
                        Commands.idle(this))
                .withName("Aim Shooter at " + angle + "degrees");
    }

    /** This command runs the shooter to shoot the note for 0.25 seconds. */
    public Command shootNoteCommand() {
        return this.runOnce(() -> {
                    topShooter.set(ShooterConstants.speakerSpeed);
                    bottomShooter.set(ShooterConstants.speakerSpeed);
                    feeder.set(0.7);
                })
                .andThen(Commands.idle(this))
                .withTimeout(0.25)
                .withName("Run the shooter");
    }

    /**
     * This command allows the operator to manually angle the shooter.
     *
     * <p>You must pass a power supplier method into this command instead of a single power, because
     * the command needs to get the power every update.
     *
     * @param rotationPowerSupplier A method that returns the rotation power, ideally a controller's
     *     joystick Y axis.
     */
    public Command manualAimCommand(DoubleSupplier rotationPowerSupplier) {
        return this.run(() -> {
                    angle.set(rotationPowerSupplier.getAsDouble());

                    // Update the calculated angle so it doesn't appear to be aiming
                    calculatedAngle.setDouble(0);
                    calculatedRotations.setDouble(0);
                })
                .withName("Manual Shooter Aim");
    }

    /**
     * This command allows the operator to manually shoot a note. For the sake of easier demos, this
     * command does not return the shooter to the bottom position.
     */
    public Command manualShootCommand() {
        return Commands.sequence(
                spinUpFlywheelsCommand(),
                shootNoteCommand(),
                this.runOnce(() -> {
                    topShooter.set(0);
                    bottomShooter.set(0);
                    feeder.set(0);
                }),
                Commands.idle(this));
    }

    /**
     * This command lowers the shooter to its bottom position and runs the feeder in reverse to eject
     * a note.
     *
     * <p>The shooter must lower and run the feeder, or else the note will remain stuck in the robot.
     */
    public Command ejectNoteCommand() {
        return this.runOnce(() -> {
                    topShooter.set(0);
                    bottomShooter.set(0);
                    feeder.set(-1);
                    setAngle(ShooterConstants.kBottomMeasureAngle, false);
                })
                .andThen(Commands.idle(this));
    }

    /**
     * Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is
     * disabled.
     */
    public void logData() {
        // Listed data
        bottomShooterSpeed.setDouble(bottomShooter.get());
        topShooterSpeed.setDouble(topShooter.get());
        feederSpeed.setDouble(feeder.get());
        angleRotations.setDouble(m_angleEncoder.getPosition());
        angleDegrees.setDouble(m_angleEncoder.getPosition() / ShooterConstants.kDegreesToRotationsConversion
                + ShooterConstants.kBottomMeasureAngle);
        tx.setDouble(LimelightHelpers.getTX("limelight-front"));
        ty.setDouble(LimelightHelpers.getTY("limelight-front"));

        // Widget data
        atSpeed.setBoolean(topShooter.getEncoder().getVelocity() < -3800);
        isAtAngle.setBoolean(isAtAngle());
        seeTag.setBoolean(tagInSight());
        txCorrect.setBoolean(Aiming.approximatelyEqual(LimelightHelpers.getTX("limelight-front"), 0, 3));
        ready.setBoolean(isReady());
    }
}
