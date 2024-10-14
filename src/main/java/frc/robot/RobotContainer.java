package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;
import frc.robot.commands.LockOnNoteCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

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
    // If you don't have both XBOX controllers connected, you can temporarily set one or both 
    // of the following variables to null. That will avoid being spammed by warnings about a
    // controller not connected.
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
        .withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * 0.06) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Set up telemetry.
    private final Telemetry logger = new Telemetry(SwerveSpeedConstants.MaxSpeed);

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Robot States */
    private double m_speedMultiplier = 1;
    private boolean m_isAmp = false;

    /* Robot State Triggers */
    private final Trigger shooterLimitSwitchPressed = new Trigger(s_Shooter::getLimitSwitch);
    private final Trigger climbLimitSwitchesPressed = new Trigger(s_Climb::getLimitSwitches);
    private final Trigger tagIsInSight = new Trigger(() -> s_Shooter.tagInSight(kAlliance));
    private final Trigger noteIsInSight = new Trigger(s_Intake::noteInSight);
    private final Trigger shooterIsReady = new Trigger(() -> s_Shooter.isReady(kAlliance));

    /* Commands */
    private final LockOnNoteCommand lockOnNoteCommmand = new LockOnNoteCommand(this, s_Swerve, driveFacingAngle);

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

        // Set the PID controller for swerve drive aiming
        driveFacingAngle.HeadingController.setPID(3, 0, 0);
        // Enable continuous angle input, so that the robot doesn't spin around to go from 180 to -180 degrees.
        driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        s_Swerve.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        s_Shooter.setDefaultCommand(
            // Stop any shooter rotation, turn off the shooter, and return to bottom position.
            Commands.sequence(
                s_Shooter.runOnce(() -> s_Shooter.rotateShooter(0)),
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0)),
                s_Shooter.startEnd(() -> s_Shooter.setAngle(ShooterConstants.kShooterMinAngle, false), () -> {})
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
        shooterLimitSwitchPressed.onTrue(s_Led.run(() -> s_Led.setColor(255, 20, 0)).withTimeout(3));

        // While a tag is in sight but the shooter is not ready, blink the LEDs red
        tagIsInSight.and(shooterIsReady.negate()).whileTrue(
            s_Led.run(() -> s_Led.blink(255, 0, 0, 300))
        );

        // When a note is in sight, the speaker tag isn't in sight, and driver is not rotating the robot, automatically lock on the note and blink the LEDs orange.
        noteIsInSight.and(tagIsInSight.negate()).whileTrue(
            Commands.parallel(
                lockOnNoteCommmand,
                s_Led.run(() -> s_Led.blink(255, 20, 0, 300))
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );

        // When the shooter is ready, turn the LEDs green
        shooterIsReady.whileTrue(
            s_Led.startEnd(() -> s_Led.setColor(0, 255, 0), () -> {})
        );

        // Configure the button bindings
        configureButtonBindings();

        // Add the target direction PID Controller to Shuffleboard
        Shuffleboard.getTab("Swerve").add("Target Direction PID", driveFacingAngle.HeadingController);

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
        https://www.padcrafter.com/?dpadRight=New+Amp+Firing+Sequence&dpadUp=Reset+Shooter+Encoder&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=&leftBumper=Prepare+intake+for+amp+spit&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&backButton=Eject+Intake&startButton=Free+a+Stuck+Note+%28on+shooter%29&rightStickClick=Lock+Speaker+using+limelight+%28old%29&rightStick=Aim+Shooter+%28Calibration+Only%29&aButton=Fire&bButton=Set+Angle%3A+Amp&xButton=Lock+Speaker&yButton=Manual+Angle+Fire&rightBumper=Intake+Amp+Spit&rightTrigger=Deploy+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&plat=0#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
        Whenever you edit a button binding, please update this URL
        */

        // Reset the field-centric heading on left bumper press. THIS WILL BREAK POSE ESTIMATION.
        // driverController.options().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        driverController.povUp().onTrue(s_Led.run(() -> s_Led.blink(0, 0, 255, 400)).withTimeout(5)); // Coopertition
        driverController.povDown().onTrue(s_Led.run(() -> s_Led.blink(255, 255, 0, 1000)).withTimeout(5)); // Amplify
        
        // Climb
        driverController.L1().and(climbLimitSwitchesPressed).whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(-1, -1), () -> {}));
        driverController.R1().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 1), () -> {}));
        driverController.square().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 0), () -> {}));
        driverController.triangle().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(0, 1), () -> {}));

        driverController.R2().onTrue(new InstantCommand(() -> m_speedMultiplier = 1, new Subsystem[0])); // no subsystems required
        driverController.L2().onTrue(new InstantCommand(() -> m_speedMultiplier = 0.2, new Subsystem[0])); // no subsystems required

        // Temporary control for finding kSlipCurrentA.
        driverController.L3().whileTrue(s_Swerve.runEnd(s_Swerve::applyIncreasingVoltage, s_Swerve::resetVoltage));

        // SysId Controls. Comment out after finished.
        driverController.touchpad().and(driverController.povLeft()).whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        driverController.touchpad().and(driverController.povRight()).whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));
        driverController.options().and(driverController.povLeft()).whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        driverController.options().and(driverController.povRight()).whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the operator controller:
            https://www.padcrafter.com/?dpadRight=New+Amp+Firing+Sequence&dpadUp=Reset+Shooter+Encoder&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=Lock+onto+a+note&leftBumper=Prepare+intake+for+amp+spit&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&backButton=Eject+Intake&startButton=Free+a+Stuck+Note+%28on+shooter%29&rightStickClick=Lock+Speaker+%28using+pose%29&rightStick=Aim+Shooter+%28Calibration+Only%29&aButton=Fire&bButton=Set+Angle%3A+Amp&xButton=Lock+Speaker&yButton=Manual+Angle+Fire&rightBumper=Intake+Amp+Spit&rightTrigger=Deploy+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&plat=0#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
            Please update this link whenever you change a button.
        */
        
        // Run intake at bottom position
        operatorController.rightTrigger().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.run(() -> s_Intake.bottomPosition())
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.6), () -> {})
            )
        );

        // Run intake at mid position
        operatorController.leftTrigger().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.run(() -> s_Intake.midPosition())
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.6), () -> {})
            )
        );


        // Lock on to speaker (old method using limelight)
        operatorController.rightStick().toggleOnTrue(
            Commands.race(
                s_Swerve.applyRequest(() -> drive
                    .withVelocityX(getVelocityX()) // Drive forward with // negative Y (forward)
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
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(getVelocityX()) // Drive forward with // negative Y (forward)
                    .withVelocityY(getVelocityY()) // Drive left with negative X (left)
                    .withTargetDirection(s_Swerve.getTargetYaw(kAlliance))),

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
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );

        // Set angle to amp
        operatorController.b().toggleOnTrue(
            Commands.parallel(
                Commands.sequence(
                    new InstantCommand(() -> m_isAmp = true, new Subsystem[0]), // no subsystems required
                    s_Shooter.startEnd(() -> s_Shooter.setAngle(52.0, false), () -> {})
                ),

                s_Led.startEnd(() -> s_Led.setColor(255, 0, 0), () -> {})
            )
        );


        // Shoot in amp or speaker, depending on the amp angle mode
        operatorController.a().onTrue(
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
                
                // Speaker:
                s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), () -> {})
                .withTimeout(0.5),
            () -> m_isAmp)
        );

        // Experimental Amp Shooting Sequence. Only use this if intake amp spitting doesn't work.
        /*operatorController.povRight().toggleOnTrue(
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
        );*/

        // Prepare intake for amp spit
        operatorController.leftBumper().toggleOnTrue(
            Commands.sequence(
                // Turn off intake and feeder
                s_Intake.runOnce(() -> s_Intake.runIntake(0, 0)),
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0)),
                // Make sure it's at the bottom
                s_Intake.startEnd(s_Intake::bottomPosition, () -> {})
                .withTimeout(0.5),
                // Spit into the ground
                Commands.parallel(
                    s_Intake.runOnce(() -> s_Intake.runIntake(-0.2, -0.2)),
                    s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, 1), () -> {})
                    .withTimeout(0.5)
                ),
                // Turn off intake and feeder, and wait for them to fully stop
                Commands.parallel(
                    s_Intake.runOnce(() -> s_Intake.runIntake(0, 0)),
                    s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, 0), () -> {})
                    .withTimeout(0.5)
                ),
                // Raise to amp position
                s_Intake.run(s_Intake::topPosition)
            )
        );
        
        // Intake Amp Spit
        operatorController.rightBumper().toggleOnTrue(
            Commands.sequence(
                s_Intake.startEnd(s_Intake::ampFastSpit, () -> {})
                .withTimeout(0.05),
                s_Intake.startEnd(s_Intake::ampPosition, () -> {})
            )
        );

        /* Manual Controls */
        // Calibration Mode
        operatorController.povLeft().toggleOnTrue(
            Commands.race( // This is race instead of parallel because one command should not run without the other.
                s_Shooter.run(() -> s_Shooter.rotateShooter(operatorController.getRightY())),
                s_Intake.run(() -> s_Intake.rotateIntake(operatorController.getLeftY()))
            )
        );
        // Only use this in calibration mode
        operatorController.povUp().onTrue(s_Shooter.runOnce(() -> s_Shooter.resetEncoders()));
        operatorController.povDown().onTrue(s_Intake.runOnce(() -> s_Intake.resetEncoders()));

        // Point Blank Shooting Angle
        /*operatorController.rightBumper()
        .onTrue(
            new InstantCommand(() -> m_isAmp = false, new Subsystem[0]) // no subsystems required
            .andThen(s_Shooter.startEnd(() -> s_Shooter.setAngle(51, false), () -> {}))
        );*/

        // Podium Shooting Angle
        /*operatorController.leftBumper()
        .onTrue(
            new InstantCommand(() -> m_isAmp = false, new Subsystem[0]) // no subsystems required
            .andThen(s_Shooter.startEnd(() -> s_Shooter.setAngle(36, false), () -> {}))
        );*/

        // Manual shoot
        operatorController.y().onTrue(
            Commands.sequence(
                // Tuck note into shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {}).withTimeout(0.05),
                // Ramp up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, 0), () -> {}).withTimeout(1.0),
                // Shoot
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, -0.5), () -> {})
                .withTimeout(0.5)
            )
        );


        // Fully eject note from intake
        operatorController.back().toggleOnTrue(
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
        
        /* New Auto Commands */
        NamedCommands.registerCommand("Aim shooter",  
            // Tuck the note into the shooter, and then ramp up and aim
            Commands.sequence(
                s_Shooter.startEnd(() -> 
                    s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> 
                    s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                .withTimeout(0.05),
                s_Shooter.runOnce(() -> s_Shooter.setAngleFromPose(s_Swerve.getPose(), kAlliance))
            )
        );

        NamedCommands.registerCommand("Run shooter",
            s_Shooter.runOnce(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5))
        );

        NamedCommands.registerCommand("Stop shooter",
            s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0))
        );

        NamedCommands.registerCommand("Lower shooter",
            s_Shooter.runOnce(() -> s_Shooter.setAngle(ShooterConstants.kShooterMinAngle, false))
        );

        NamedCommands.registerCommand("Enable auto aim",
            s_Swerve.runOnce(() -> s_Swerve.setAutoRotationOverride(true))
        );

        NamedCommands.registerCommand("Disable auto aim",
            s_Swerve.runOnce(() -> s_Swerve.setAutoRotationOverride(false))
        );

        NamedCommands.registerCommand("Run intake", 
            Commands.parallel(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.runOnce(s_Intake::midPosition)
                ),
                s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, -0.6))
            )
        );

        NamedCommands.registerCommand("Stop intake", 
            Commands.sequence(
                s_Intake.runOnce(() -> s_Intake.rotateIntake(0)),
                s_Intake.runOnce(() -> s_Intake.runIntake(0, 0))
            )
        );

        NamedCommands.registerCommand("Raise intake",
            s_Intake.runOnce(s_Intake::topPosition)
        );


        /* Old Auto Commands */
        NamedCommands.registerCommand("Shoot", 
            //Shoot
            new RunCommand(() -> s_Shooter.runShooter(0.7, 0.7, -1), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Feeder", 
            //Shoot
            new RunCommand(() -> s_Shooter.runShooter(0, 0, -1), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Shoot Start",  
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(0.7, 0.7, 0), s_Shooter).withTimeout(1.0),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.7, 0.7, -0.5), s_Shooter).withTimeout(1)
            )
        );

        NamedCommands.registerCommand("Angle 25", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(40, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 30", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(30, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 32", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(32, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 35", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(35, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 37", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(37, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 45", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(45, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 51", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(51, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 42", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(42, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 40", 
            //Shoot
            new RunCommand(() -> s_Shooter.setAngle(40, false), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Feeder On", 
                //Shoot
                new RunCommand(() -> s_Shooter.runFeed(-1), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Shoot Stop", 
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.0, 0.0, 0.0), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Intake Down", 
                new RunCommand(() -> s_Intake.bottomPosition(), s_Intake).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Intake Up", 
                //Shoot
                new RunCommand(() -> s_Intake.topPosition(), s_Intake).withTimeout(0.5)
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
        double velocityX = -driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * m_speedMultiplier;
        return velocityX;
    }

    public double getVelocityY() {
        double velocityY = -driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * m_speedMultiplier;
        return velocityY;
    }

    public double getRotationalRate() {
        double rotationalRate = -driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate * m_speedMultiplier;
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
        s_Shooter.logData();
        s_Intake.logData();
    }
}