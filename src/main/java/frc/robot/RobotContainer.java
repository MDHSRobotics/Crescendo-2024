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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;
import frc.robot.commands.LockOnNoteCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
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
    /*private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt()
        .withModuleDirection(Rotation2d.fromDegrees(0));*/

    // Set up telemetry.
    private final Telemetry logger = new Telemetry();

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Robot States */
    private boolean m_isAmp = false;

    /* Robot State Triggers */
    private final Trigger shooterLimitSwitchPressed = new Trigger(s_Shooter::getLimitSwitch);
    private final Trigger climbLimitSwitchesPressed = new Trigger(s_Climb::getLimitSwitches);
    private final Trigger tagIsInSight = new Trigger(() -> s_Shooter.tagInSight(kAlliance));
    //private final Trigger noteIsInSight = new Trigger(s_Intake::noteInSight);
    private final Trigger shooterIsReady = new Trigger(() -> s_Shooter.isReady(kAlliance));

    /* Commands */
    // private final LockOnNoteCommand lockOnNoteCommmand = new LockOnNoteCommand(this, s_Swerve, driveFacingAngle);

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

        s_Shooter.setDefaultCommand(
            // Stop any shooter rotation, turn off the shooter, and return to bottom position.
            Commands.sequence(
                s_Shooter.runOnce(() -> s_Shooter.rotateShooter(0)),
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0)),
                s_Shooter.startEnd(() -> s_Shooter.setAngle(ShooterConstants.kBottomMeasureAngle, false), () -> {})
            )
        );

        s_Intake.setDefaultCommand(
            // Stop any intake rotation, turn off intake, and return to top position.
            Commands.sequence(
                s_Intake.runOnce(() -> s_Intake.rotateIntake(0)),
                s_Intake.runOnce(() -> s_Intake.runIntake(0, 0)),
                s_Intake.startEnd(() -> s_Intake.topPosition(), () -> {})
            )
        );

        s_Led.setDefaultCommand(
            s_Led.run(()-> s_Led.rainbow())
        );

        s_Climb.setDefaultCommand(
            s_Climb.startEnd(() -> s_Climb.runClimb(0,0), () -> {})
        );


        /* Trigger-Activated Commands */
        // LEDs glow orange for 3 secs whenever a note is picked up.
        shooterLimitSwitchPressed.onTrue(
            s_Led.startEnd(() -> s_Led.setColor(255, 20, 0), () -> {})
                .withTimeout(3)
        );

        // While a tag is in sight but the shooter is not ready, blink the LEDs red
        tagIsInSight.and(shooterIsReady.negate()).whileTrue(
            s_Led.run(() -> s_Led.blink(255, 0, 0, 300))
        );

        // When a note is in sight, the speaker tag isn't in sight, and driver is not rotating the robot, automatically lock on the note and blink the LEDs orange.
        // According to https://www.chiefdelphi.com/t/what-are-your-programming-horror-stories/473439/15, we also need to make sure this does not run in autonomous.
        // This is disabled until we get the back camera back on.
        /*noteIsInSight.and(tagIsInSight.negate()).and(RobotModeTriggers.autonomous().negate()).whileTrue(
            Commands.parallel(
                lockOnNoteCommmand,
                s_Led.run(() -> s_Led.blink(255, 20, 0, 300))
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );*/

        // When the shooter is ready, turn the LEDs green
        shooterIsReady.whileTrue(
            s_Led.startEnd(() -> s_Led.setColor(0, 255, 0), () -> {})
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
        https://www.padcrafter.com/index.php?templates=Driver+Controller&leftBumper=Climb+Down&dpadRight=Lock+onto+Stage+%28left+side%29&dpadLeft=Lock+onto+Stage+%28right+side%29&aButton=&yButton=Right+Climb+Up&dpadDown=Lock+onto+Stage+%28middle%29&dpadUp=&xButton=Left+Climb+Up&bButton=&leftStick=Field+Oriented+Drive&rightStick=Rotate+Robot&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=&leftTrigger=%28Hold%29+Drive+slow&rightBumper=Climb+Up&startButton=Reset+Field+Oriented+Drive&plat=1&backButton=&rightStickClick=
        Whenever you edit a button binding, please update this URL
        */

        driverController.options().onTrue(
            s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative())
        );

        // Slow mode
        driverController.R2().whileTrue(
            s_Swerve.applyRequest(() -> driveSlow
                .withVelocityX(getVelocityX() * 0.5)
                .withVelocityY(getVelocityY() * 0.5)
                .withRotationalRate(getRotationalRate() * 0.5)
            )
        );

        // Stage aiming
        driverController.povRight().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_LEFT, kAlliance))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );

        driverController.povLeft().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_RIGHT, kAlliance))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );

        driverController.povDown().onTrue(
            Commands.sequence(
                // Reset the PID controller
                s_Swerve.runOnce(() -> driveFacingAngle.HeadingController.reset()),
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX() * 0.5)
                    .withVelocityY(getVelocityY() * 0.5)
                    .withTargetDirection(s_Swerve.getTargetDirection(HeadingTargets.STAGE_MIDDLE, kAlliance))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );
        
        // Climb
        driverController.L1().and(climbLimitSwitchesPressed.negate()).whileTrue(
            s_Climb.startEnd(() -> s_Climb.runClimb(-1, -1), () -> {})
        );

        driverController.R1().whileTrue(
            s_Climb.startEnd(() -> s_Climb.runClimb(1, 1), () -> {})
        );

        driverController.square().whileTrue(
            s_Climb.startEnd(() -> s_Climb.runClimb(1, 0), () -> {})
        );

        driverController.triangle().whileTrue(
            s_Climb.startEnd(() -> s_Climb.runClimb(0, 1), () -> {})
        );
        

        // SysId Controls. Comment out stage aiming controls before you use this.
        // driverController.povUp().whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        // driverController.povDown().whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));
        // driverController.povUp().whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        // driverController.povDown().whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the operator controller:
            https://www.padcrafter.com/?dpadRight=&dpadUp=&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=Set+angle+to+amp&leftBumper=&leftTrigger=%28Hold%29+Deploy+Intake+lower&dpadLeft=&dpadDown=&backButton=%28Hold%29+Eject+Intake&startButton=%28Hold%29+Get+note+off+the+shooter%27s+top&rightStickClick=Lock+Speaker+%28point+blank%29&rightStick=Aim+Shooter+%28Calibration+Only%29&aButton=Fire&bButton=Lock+Speaker+%28limelight+only%29&xButton=Lock+Speaker&yButton=Lock+Amp+%28for+passing%29&rightBumper=&rightTrigger=%28Hold%29+Deploy+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&plat=0#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
            Please update this link whenever you change a button.
        */
        
        // Run intake at mid position
        operatorController.rightTrigger().whileTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.startEnd(s_Intake::midPosition, () -> {})
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.6), () -> {})
            )
        );
        
        // Run intake at bottom position in case mid isn't low enough
        operatorController.leftTrigger().whileTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.startEnd(s_Intake::bottomPosition, () -> {})
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.6), () -> {})
            )
        );


        // Lock on to speaker (old method using limelight)
        operatorController.b().toggleOnTrue(
            Commands.race(
                s_Swerve.applyRequest(() -> drive
                    .withVelocityX(getVelocityX()) // Drive forward with negative Y (forward)
                    .withVelocityY(getVelocityY()) // Drive left with negative X (left)
                    .withRotationalRate(s_Swerve.calculateTagRotationalRate())),

                Commands.sequence(
                    // Set firing mode to speaker
                    new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                    // Rev up the shooter
                    s_Shooter.startEnd(() -> 
                        s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                        s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                    .withTimeout(0.05),

                    // Angle the shooter
                    s_Shooter.run(() -> s_Shooter.setAngleFromLimelight(kAlliance))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
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

                Commands.sequence(
                    // Set firing mode to speaker
                    new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                    // Rev up the shooter
                    s_Shooter.startEnd(() -> 
                        s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                        s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                    .withTimeout(0.05),

                    // Angle the shooter
                    s_Shooter.run(() -> s_Shooter.setAngleFromPose(s_Swerve.getPose(), kAlliance))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > 0.75)
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

                Commands.sequence(
                    // Set firing mode to speaker
                    new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                    // Rev up the shooter
                    s_Shooter.startEnd(() -> 
                        s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                        s_Shooter.runShooter(ShooterConstants.passingSpeed, ShooterConstants.passingSpeed, 0))
                    .withTimeout(0.05),

                    // Angle the shooter
                    s_Shooter.startEnd(() -> s_Shooter.setAngle(ShooterConstants.passingAngle, false), () -> {})
                )
            ).until(() -> Math.abs(driverController.getRightX()) > 0.75)
        );

        // Set angle to amp
        operatorController.leftStick().toggleOnTrue(
            Commands.parallel(
                new InstantCommand(() -> m_isAmp = true, new Subsystem[0]), // no subsystems required
                s_Shooter.startEnd(() -> s_Shooter.setAngle(ShooterConstants.ampAngle, false), () -> {}),
                s_Led.startEnd(() -> s_Led.setColor(255, 0, 0), () -> {})
            )
        );


        // Shoot in amp or speaker, depending on the amp angle mode
        operatorController.a().onTrue(
            Commands.race(
                // Prevent swerve movement to eliminate momentum.
                s_Swerve.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
                ),
                Commands.either(
                    // Amp:
                    Commands.sequence(
                        // Tuck note into shooter
                        s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {})
                            .withTimeout(0.05),
                        // Ramp up
                        s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, 0), () -> {})
                            .withTimeout(1.0),
                        // Shoot into amp
                        s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, -0.5), () -> {})
                            .withTimeout(0.2)
                    ),
                    // OR
                    // Speaker:
                    s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.7), () -> {})
                        .withTimeout(0.25),
                    () -> m_isAmp
                )
            )
        );

        // Experimental Amp Shooting Sequence.
        operatorController.leftBumper().toggleOnTrue(
            Commands.sequence(
                // Tuck note into shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {})
                .withTimeout(0.05),
                // Ramp up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.2, 0.2, 0), () -> {})
                .withTimeout(1.0),
                // Slowly raise the pitch of the shooter until it reaches the correct angle
                s_Shooter.startEnd(() -> s_Shooter.rotateShooter(0.2), () -> {})
                .until(() -> s_Shooter.getAngleDegrees() >= 52.0),
                // Shoot into amp, and stop before the shooter raises too high up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.2, 0.2, -0.5), () -> {})
                .until(() -> s_Shooter.getAngleDegrees() >= 80.0)
            )
        );
        

        /* Manual Controls */
        // Calibration Mode
        // Warning: We usually set the starting position by hand while the robot is off.
        // Only use this if you ABSOLUTELY do not have time to do it by hand.
        operatorController.povLeft().toggleOnTrue(
            Commands.parallel(
                s_Shooter.run(() -> s_Shooter.rotateShooter(operatorController.getRightY())),
                s_Intake.run(() -> s_Intake.rotateIntake(operatorController.getLeftY()))
            )
        );
        // Only use this in calibration mode
        operatorController.povUp().onTrue(s_Shooter.runOnce(() -> s_Shooter.resetEncoders()));
        operatorController.povDown().onTrue(s_Intake.runOnce(() -> s_Intake.resetEncoders()));

        // Point Blank Shooting Angle
        operatorController.rightStick().toggleOnTrue(
            Commands.sequence(
                // Set firing mode to speaker
                new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                // Rev up the shooter
                s_Shooter.startEnd(() -> 
                    s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                    s_Shooter.runShooter(ShooterConstants.passingSpeed, ShooterConstants.passingSpeed, 0))
                .withTimeout(0.05),

                // Angle the shooter
                s_Shooter.startEnd(() -> s_Shooter.setAngle(51, false), () -> {})
            )
        );

        // Podium Shooting Angle
        /*operatorController.leftBumper()
        .onTrue(
            new InstantCommand(() -> m_isAmp = false, new Subsystem[0]) // no subsystems required
            .andThen(s_Shooter.startEnd(() -> s_Shooter.setAngle(36, false), () -> {}))
        );*/

        // Manual shoot
        /*operatorController.y().onTrue(
            Commands.sequence(
                // Tuck note into shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {}).withTimeout(0.05),
                // Ramp up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, 0), () -> {}).withTimeout(1.0),
                // Shoot
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, -0.5), () -> {})
                .withTimeout(0.5)
            )
        );*/


        // Fully eject note from intake
        operatorController.back().whileTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(-1, -1)),
                    s_Intake.run(s_Intake::ejectPosition)
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, 1), () -> {})
            )
        );

        // Free a stuck note on the top of the robot
        operatorController.start().whileTrue(
            s_Shooter.startEnd(() -> s_Shooter.runShooter(-1,-1,0), () -> {})
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
                // Tuck the note into the shooter, then rev up
                s_Shooter.startEnd(() -> 
                    s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                    s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0)
                ).withTimeout(0.1),
                // Angle the shooter
                s_Shooter.run(() -> s_Shooter.setAngleFromPose(s_Swerve.getPose(), kAlliance))
                 .withTimeout(0.75),
                // Run the shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.7), () -> {})
                 .withTimeout(0.25),
                // Lower the shooter
                s_Shooter.runOnce(() -> s_Shooter.setAngle(ShooterConstants.kBottomMeasureAngle, false)),
                // Turn off the shooter
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0))
            )
        );

        NamedCommands.registerCommand("Run intake", 
            Commands.parallel(
                // Turn on the feeder
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, -1)),
                // Run the intake and lower it
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.startEnd(s_Intake::bottomPosition, () -> {})
                        .withTimeout(0.5)
                )
            )
        );

        NamedCommands.registerCommand("Stop intake", 
            Commands.parallel(
                // Turn off the feeder
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0)),
                // Turn off the intake and raise it
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(0, 0)),
                    s_Intake.startEnd(s_Intake::topPosition, () -> {})
                        .withTimeout(0.5)
                )
            )
        );

        NamedCommands.registerCommand("Run intake only", 
            Commands.parallel(
                // Turn on the feeder
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, -1)),
                // Run the intake
                s_Intake.runOnce(() -> s_Intake.runIntake(1, 1))
            )
        );

        NamedCommands.registerCommand("Stop intake only", 
            Commands.parallel(
                // Turn off the feeder
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0)),
                // Turn off the intake
                s_Intake.startEnd(() -> s_Intake.runIntake(0, 0), () -> {})
                    .withTimeout(0.5)
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

    public void setStartingPosition(Pose2d startingPosition) {
        s_Swerve.seedFieldRelative(startingPosition);
    }

    public void setOperatorPerspective(Rotation2d fieldDirection) {
        s_Swerve.setOperatorPerspectiveForward(fieldDirection);
    }

    public void logSubsystemData() {
        s_Swerve.logData();
        s_Shooter.logData(kAlliance);
        s_Intake.logData();
        s_Climb.logData();
    }
}