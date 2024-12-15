package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakePositions;
import frc.robot.subsystems.Swerve.HeadingTargets;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Robot Alliance
    public Alliance kAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    
    /* Controllers */
    private final CommandPS4Controller driverController = new CommandPS4Controller(0); 
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* Subsystems */
    private final Swerve s_Swerve = TunerConstants.DriveTrain; // My drivetrain
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Climb s_Climb = new Climb();
    private final LED s_Led = new LED();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * 0.06) // Add a 6% deadband to prevent joystick drift
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
    
    // Slow drive has half the deadband to allow for lower minimum speed.
    private final SwerveRequest.FieldCentric driveSlow = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband * 0.5)
        .withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * 0.06 * 0.5)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Point wheels in one direction in preparation for SysId testing.
    private final SwerveRequest.PointWheelsAt pointWheelsForward = new SwerveRequest.PointWheelsAt()
        .withModuleDirection(Rotation2d.fromDegrees(0));

    // Set up telemetry.
    private final Telemetry logger = new Telemetry();

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Robot State Triggers */
    private final Trigger climbLimitSwitchesPressed = new Trigger(s_Climb::getLimitSwitches);
    private final Trigger tagIsInSight = new Trigger(() -> s_Shooter.tagInSight());
    private final Trigger shooterIsReady = new Trigger(() -> s_Shooter.isReady());

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Register the autonomous commands for Pathplanner
        registerPathplannerCommands();

        /* Default Commands */
        if (driverController != null) {
            s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
                    s_Swerve.applyRequest(() -> drive
                        .withVelocityX(getVelocityX()) // Forward and backward speed
                        .withVelocityY(getVelocityY()) // Left and right speed
                        .withRotationalRate(getRotationalRate()) // Rotation speed
                    )
                    .withName("Default Drive")
            );
        }

        // Set the PID controller for swerve drive aiming.
        // If you use kI in any PID controller, or kD in a Phoenix PID Controller, you must call reset() before you start aiming to prevent a large spike in output.
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html#resetting-the-controller
        driveFacingAngle.HeadingController.setPID(3, 0, 0);
        // Enable continuous angle input, so that the robot doesn't spin around to go from 180 to -180 degrees.
        driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Add the target direction PID Controller to Shuffleboard
        Shuffleboard.getTab("Swerve").add("Target Direction PID", driveFacingAngle.HeadingController);

        s_Swerve.registerTelemetry(logger::telemeterize);

        s_Shooter.setDefaultCommand(s_Shooter.disableShooterCommand());

        s_Intake.setDefaultCommand(s_Intake.disableIntakeCommand(true));

        s_Led.setDefaultCommand(s_Led.rainbowCommand());

        s_Climb.setDefaultCommand(s_Climb.runClimbCommand(0, 0).withName("Disable Climb"));


        /* Trigger-Activated Commands */
        // While a tag is in sight but the shooter is not ready, blink the LEDs red
        tagIsInSight.and(shooterIsReady.negate()).whileTrue(
            s_Led.blinkCommand(255, 0, 0, 300)
            .withName("Blink LEDs Red")
        );

        // When the shooter is ready, turn the LEDs green
        shooterIsReady.whileTrue(
            s_Led.setColorCommand(0, 255, 0)
            .withName("Green LEDs")
        );

        // Configure the button bindings
        configureButtonBindings();

        /* Auto Chooser */
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        Shuffleboard.getTab("Main").add("Select your Auto:", autoChooser).withSize(2, 1);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        if (driverController != null) {
            configureDriverButtonBindings();
        }

        if (operatorController != null) {
            configureOperatorButtonBindings();
        }

    }

    private void configureDriverButtonBindings() {

        /* Driver Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the driver controller:
        https://www.padcrafter.com/index.php?templates=Driver+Controller&leftBumper=Climb+Down&dpadRight=Right+Climb+Up&dpadLeft=Left+Climb+Up&aButton=Lock+onto+Stage+%28middle%29&yButton=Lock+onto+Source&dpadDown=&dpadUp=&xButton=Lock+onto+Stage+%28right+side%29&bButton=Lock+onto+Stage+%28left+side%29&leftStick=Field+Oriented+Drive&rightStick=Rotate+Robot&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=&leftTrigger=%28Hold%29+Drive+slow&rightBumper=Climb+Up&startButton=Reset+Field+Oriented+Drive&plat=1&backButton=&rightStickClick=
        Whenever you edit a button binding, please update this URL
        */

        driverController.options().onTrue(
            s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative())
            .withName("Reset Robot Perspective")
        );

        // Slow mode
        driverController.L2().whileTrue(
            s_Swerve.applyRequest(() -> driveSlow
                .withVelocityX(getVelocityX() * 0.5)
                .withVelocityY(getVelocityY() * 0.5)
                .withRotationalRate(getRotationalRate() * 0.5)
            )
            .withName("Drive Slow")
        );
        
        // Source aiming
        driverController.triangle().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(driveFacingAngle.HeadingController::reset),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.SOURCE, kAlliance))
                )
            )
            .until(this::driverAttempedToRotate)
            .withName("Drive Slow Facing Source")
        );

        // Stage aiming
        driverController.circle().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_LEFT, kAlliance))
                )
            )
            .until(this::driverAttempedToRotate)
            .withName("Drive Slow Facing the Stage's Left Side")
        );

        driverController.square().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_RIGHT, kAlliance))
                )
            )
            .until(this::driverAttempedToRotate)
            .withName("Drive Slow Facing the Stage's Right Side")
        );

        driverController.cross().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_MIDDLE, kAlliance))
                )
            )
            .until(this::driverAttempedToRotate)
            .withName("Drive Slow Facing the Stage's Middle/Back Side")
        );
        
        // Climb
        driverController.L1().and(climbLimitSwitchesPressed.negate()).whileTrue(
            s_Climb.runClimbCommand(-1, -1).withName("Lower Both Climbs")
        );

        driverController.R1().whileTrue(
            s_Climb.runClimbCommand(1, 1).withName("Raise Both Climbs")
        );

        driverController.povLeft().whileTrue(
            s_Climb.runClimbCommand(1, 0).withName("Raise the Left Climb")
        );

        driverController.povRight().whileTrue(
            s_Climb.runClimbCommand(0, 1).withName("Raise the Right Climb")
        );

        // Run wheel radius calculation
        //driverController.share().whileTrue(new WheelRadiusCharacterization(s_Swerve));        

        // Point wheels forward in preparation for SysId
        /*driverController.touchpad().whileTrue(
            s_Swerve.applyRequest(() -> pointWheelsForward)
        );

        // SysId Controls
        driverController.povLeft().whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        driverController.povRight().whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));
        driverController.povUp().whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        driverController.povDown().whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));*/
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the operator controller:
            https://www.padcrafter.com/?dpadRight=Manual+Note+Shot&dpadUp=&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=Set+angle+to+amp&leftBumper=&leftTrigger=%28Toggle%29+Deploy+Intake+lower&dpadLeft=Toggle+manual+shooter+aim&dpadDown=&backButton=%28Toggle%29+Eject+Intake&startButton=%28Toggle%29+Get+note+off+the+shooter%27s+top&rightStickClick=Lock+Speaker+%28point+blank%29&rightStick=Aim+Shooter+%28must+be+toggled%29&aButton=Fire&bButton=Lock+Speaker+%28limelight+only%29&xButton=Lock+Speaker&yButton=Lock+Amp+%28for+passing%29&rightBumper=&rightTrigger=%28Toggle%29+Deploy+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&plat=0#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
            Please update this link whenever you change a button.
        */
        
        // Run intake at mid position
        operatorController.rightTrigger().toggleOnTrue(
            Commands.race(
                s_Intake.intakeNoteCommand(IntakePositions.MID),
                s_Shooter.intakeNoteCommand()
            )
            .withName("Run Intake at Mid Position")
        );
        
        // Run intake at bottom position in case mid isn't low enough
        operatorController.leftTrigger().toggleOnTrue(
            Commands.race(
                s_Intake.intakeNoteCommand(IntakePositions.BOTTOM),
                s_Shooter.intakeNoteCommand()
            )
            .withName("Run Intake at Bottom Position")
        );


        // Lock on to speaker (old method using limelight)
        operatorController.b().toggleOnTrue(
            Commands.race(
                s_Swerve.applyRequest(() -> drive
                    .withVelocityX(getVelocityX()) // Drive forward with negative Y (forward)
                    .withVelocityY(getVelocityY()) // Drive left with negative X (left)
                    .withRotationalRate(s_Swerve.calculateTagRotationalRate())
                ),
                s_Shooter.aimShooterCommand()
            ).until(this::driverAttempedToRotate)
            .withName("Lock onto Speaker with Limelight")
        );
        
        // Lock on to speaker (new method using pose estimation).
        operatorController.x().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    // Reset the PID controller
                    s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                    s_Swerve.applyRequest(() -> driveFacingAngle
                        .withVelocityX(getVelocityX())
                        .withVelocityY(getVelocityY())
                        .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.SPEAKER, kAlliance)))
                ),
                s_Shooter.aimShooterWithPoseCommand(s_Swerve::getPose)
            ).until(this::driverAttempedToRotate)
            .withName("Lock onto Speaker with Pose")
        );

        // Lock on to amp area (for note passing).
        operatorController.y().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    // Reset the PID controller
                    s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                    s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX()) // Drive forward with negative Y (forward)
                    .withVelocityY(getVelocityY()) // Drive left with negative X (left)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.AMP_AREA, kAlliance)))
                ),
                s_Shooter.aimShooterWithAngleCommand(ShooterConstants.passingAngle)
            ).until(this::driverAttempedToRotate)
            .withName("Lock onto passing area")
        );


        // Shoot in speaker
        operatorController.a().onTrue(
            Commands.race(
                // Prevent swerve movement to eliminate momentum.
                s_Swerve.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ),
                s_Shooter.shootNoteCommand()
            )
            .withName("Shoot Note")
        );
        

        /* Manual Controls */
        // Enable manual aim for the shooter
        operatorController.povLeft().toggleOnTrue(
           s_Shooter.manualAimCommand(operatorController::getRightY)
        );

        // Manually rev up and shoot a note
        operatorController.povRight().onTrue(
            s_Shooter.manualShootCommand()
        );

        // Point Blank Shooting Angle
        operatorController.rightStick().toggleOnTrue(
            s_Shooter.aimShooterWithAngleCommand(51)
        );

        // Fully eject note from intake
        operatorController.back().toggleOnTrue(
            Commands.race(
                s_Intake.ejectNoteCommand(),
                s_Shooter.ejectNoteCommand()
            )
            .withName("Eject Note")
        );
    }


    // Register autonomous commands
    private void registerPathplannerCommands()
    {
        // IMPORTANT: In autonomous, the default subsystem commands do not get scheduled.
        // Also, commands MUST have an end, or PathPlanner will not continue.
        // ALSO, Triggers can still activate in autonomous, so do not add any Trigger-activated commands that require subsystems used in autonomous.
        
        /* New Auto Commands */
        NamedCommands.registerCommand("Shoot note",  
            Commands.sequence(
                // Angle the shooter
                s_Shooter.aimShooterWithPoseCommand(s_Swerve::getPose)
                    .withTimeout(0.75),
                // Run the shooter
                s_Shooter.shootNoteCommand(),
                // Disable the shooter
                s_Shooter.disableShooterCommand()
            )
        );

        NamedCommands.registerCommand("Run intake", 
            Commands.parallel(
                // Turn on the feeder
                s_Shooter.intakeNoteCommand(),
                // Run the intake and lower it
                s_Intake.intakeNoteCommand(IntakePositions.BOTTOM)
            ).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Stop intake", 
            Commands.parallel(
                // Turn off the feeder
                s_Shooter.disableShooterCommand(),
                // Turn off the intake and raise it
                s_Intake.disableIntakeCommand(true)
            ).withTimeout(0.5)
        );

        // This command expects the intake to already be at the bottom position, so no time is wasted with waiting.
        NamedCommands.registerCommand("Run intake only", 
            Commands.parallel(
                // Turn on the feeder
                s_Shooter.intakeNoteCommand(),
                // Run the intake
                s_Intake.intakeNoteCommand(IntakePositions.BOTTOM)
            )
        );

        NamedCommands.registerCommand("Stop intake only", 
            Commands.parallel(
                // Turn off the feeder
                s_Shooter.disableShooterCommand(),
                // Turn off the intake without returning to top position
                s_Intake.disableIntakeCommand(false)
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public double getVelocityX() {
        double velocityX = -driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed;
        return velocityX;
    }

    public double getVelocityY() {
        double velocityY = -driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed;
        return velocityY;
    }

    public double getRotationalRate() {
        double rotationalRate = -driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate;
        return rotationalRate;
    }

    public boolean driverAttempedToRotate() {
        boolean deadbandPassed = Math.abs(driverController.getRightX()) > Constants.stickDeadband;
        return deadbandPassed;
    }

    public void setStartingPosition(Pose2d startingPosition) {
        s_Swerve.seedFieldRelative(startingPosition);
    }

    public void setOperatorPerspective(Rotation2d fieldDirection) {
        s_Swerve.setOperatorPerspectiveForward(fieldDirection);
    }

    public void logSubsystemData() {
        s_Swerve.logData();
        s_Shooter.logData();
        s_Intake.logData();
        s_Climb.logData();
    }
}