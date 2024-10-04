package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
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
    
    /* Limit Switches */
    DigitalInput shooterLimitSwitch = new DigitalInput(ShooterConstants.kLimitSwitchID);
    DigitalInput climbLimitSwitch = new DigitalInput(ClimbConstants.kLimitSwitchID);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * Constants.stickDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // Set up telemetry.
    private final Telemetry logger = new Telemetry(SwerveSpeedConstants.MaxSpeed);

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Robot States */
    private boolean m_slowMode = false;
    private boolean m_isAmp = false;
    private boolean m_autoshoot = false;

    /* Robot State Triggers */
    private Trigger shooterLimitSwitchPressed = new Trigger(shooterLimitSwitch::get);
    private Trigger climbLimitSwitchPressed = new Trigger(climbLimitSwitch::get);
    private Trigger ampModeActivated = new Trigger(() -> m_isAmp);
    private Trigger tagIsInSight = new Trigger(s_Shooter::tagInSight);
    private Trigger shooterIsReady = new Trigger(s_Shooter::isReady);
    private Trigger autoshootEnabled = new Trigger(() -> m_autoshoot);

    public Alliance kAlliance;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Default Commands */
        if (driverController != null) {
            s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
                    s_Swerve.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Forward and backward speed
                        .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Left and right speed
                        .withRotationalRate(-driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate * (m_slowMode ? 0.2 : 1.0))) // Rotation speed
            );
        }

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

        // When a tag is in sight but the shooter is not ready, blink the LEDs red
        tagIsInSight.and(shooterIsReady.negate()).toggleOnTrue(
            s_Led.run(() -> s_Led.blink(255, 0, 0, 300))
        );

        // When the shooter is ready and autoshoot is enabled, then shoot
        shooterIsReady.and(autoshootEnabled).onTrue(
            s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), () -> {})
            .withTimeout(0.5)
        );

        // When the shooter is ready, turn the LEDs green
        shooterIsReady.toggleOnTrue(
            s_Led.startEnd(() -> s_Led.setColor(0, 255, 0), () -> {})
        );

        // Configure the button bindings
        configureButtonBindings();

        // Register the autonomous commands for Pathplanner
        registerPathplannerCommands();

        /* Auto Chooser */
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        Shuffleboard.getTab("Main").add("Select your Auto:", autoChooser).withSize(2, 1);

        // Save the current alliance for use in Robot.java
        kAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
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
        https://www.padcrafter.com/?dpadRight=New+Amp+Firing+Sequence&dpadUp=Reset+Shooter+Encoder&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=Lock+Note&leftBumper=Prepare+intake+for+amp+spit&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&backButton=Eject+Intake&startButton=Free+a+Stuck+Note+%28on+shooter%29&rightStickClick=Lock+Speaker+%28using+pose%29&rightStick=Aim+Shooter+%28Calibration+Only%29&aButton=Fire&bButton=Set+Angle%3A+Amp&xButton=Lock+Speaker&yButton=Manual+Angle+Fire&rightBumper=Intake+Amp+Spit&rightTrigger=Deploy+Intake&templates=Controller+Scheme+1#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
        Whenever you edit a button binding, please update this URL
        */

        // Brake the robot by crossing the wheels.
        // If the operator locks onto the speaker, the wheels will stop braking.
        driverController.cross().whileTrue(s_Swerve.applyRequest(() -> brake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Reset the field-centric heading on left bumper press. For pose estimation to start working again, drive to an Apriltag. Do not uncomment this until you finish SysId.
        //driverController.options().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        driverController.povUp().onTrue(s_Led.run(() -> s_Led.blink(0, 0, 255, 400)).withTimeout(5)); // Coopertition
        driverController.povDown().onTrue(s_Led.run(() -> s_Led.blink(255, 255, 0, 1000)).withTimeout(5)); // Amplify
        
        // Climb
        driverController.L1().and(climbLimitSwitchPressed).whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(-1, -1), () -> {}));
        driverController.R1().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 1), () -> {}));
        driverController.square().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 0), () -> {}));
        driverController.triangle().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(0, 1), () -> {}));

        driverController.R2().onTrue(new InstantCommand(() -> m_slowMode = false, new Subsystem[0])); // no subsystems required
        driverController.L2().onTrue(new InstantCommand(() -> m_slowMode = true, new Subsystem[0])); // no subsystems required

        // Temporary control for finding kSlipCurrentA. Remove after finished.
        driverController.L3().whileTrue(s_Swerve.run(s_Swerve::applyIncreasingVoltage));

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
            https://www.padcrafter.com/?dpadRight=New+Amp+Firing+Sequence&dpadUp=Reset+Shooter+Encoder&leftStick=Aim+Intake+%28Calibration+Only%29&leftStickClick=&leftBumper=Prepare+intake+for+amp+spit&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&backButton=Eject+Intake&startButton=Free+a+Stuck+Note+%28on+shooter%29&rightStickClick=Lock+Speaker+%28using+pose%29&rightStick=Aim+Shooter+%28Calibration+Only%29&aButton=Fire&bButton=Set+Angle%3A+Amp&xButton=Lock+Speaker&yButton=Manual+Angle+Fire&rightBumper=Intake+Amp+Spit&rightTrigger=Deploy+Intake#?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Manual+Angle+Fire&leftStickClick=Toggle+Auto+Shoot
            Please update this link whenever you change a button.
        */
        
        // Run intake at bottom position
        operatorController.rightTrigger().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.run(() -> s_Intake.bottomPosition())
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.7), () -> {})
            )
        );

        // Run intake at mid position
        operatorController.leftTrigger().toggleOnTrue(
            Commands.race(
                Commands.sequence(
                    s_Intake.runOnce(() -> s_Intake.runIntake(1, 1)),
                    s_Intake.run(() -> s_Intake.midPosition())
                ),
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.7), () -> {})
            )
        );


        // Lock on to speaker (using limelight)
        operatorController.x().toggleOnTrue(
            Commands.race(
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with // negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                    .withTargetDirection(Rotation2d.fromDegrees(s_Swerve.getRobotYaw() - LimelightHelpers.getTX("limelight-front")))),

                Commands.sequence(
                    // Set firing mode to speaker
                    new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                    // Rev up the shooter
                    s_Shooter.startEnd(() -> 
                        s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                        s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                    .withTimeout(0.05),

                    // Angle the shooter
                    s_Shooter.run(() -> s_Shooter.setAngleFromLimelight())
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );
        
        // Lock on to speaker (using pose estimation). Do not use until you have verified the accuracy of pose data in AdvantageScope.
        /*operatorController.rightStick().toggleOnTrue(
            Commands.race(
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with // negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                    .withTargetDirection(s_Swerve.getTargetYaw())),

                Commands.sequence(
                    // Set firing mode to speaker
                    new InstantCommand(() -> m_isAmp = false, new Subsystem[0]), // no subsystems required
                    // Rev up the shooter
                    s_Shooter.startEnd(() -> 
                        s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                        s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                    .withTimeout(0.05),

                    // Angle the shooter
                    s_Shooter.run(() -> s_Shooter.setAngleFromPose(s_Swerve.getPose()))
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );*/

        // Lock onto a note (using limelight-back). Do not use until you have connected the limelight and renamed it to limelight-back.
        /*operatorController.rightStick().toggleOnTrue(
            s_Swerve.applyRequest(() -> driveFacingAngle
                .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                .withTargetDirection(Rotation2d.fromDegrees(s_Swerve.getRobotYaw() - LimelightHelpers.getTX("limelight-back")))

            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );*/

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

        // Toggle Auto Shoot. Disabled until we have more buttons available.
        /*operatorController.leftStick().onTrue(
            new InstantCommand(() -> m_autoshoot = !m_autoshoot, new Subsystem[0]) // no subsystems required
        );*/


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
            ampModeActivated)
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
                    s_Intake.run(s_Intake::midPosition)
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

        NamedCommands.registerCommand("Rev Up the Shooter",  
            // Tuck the note into the shooter, and then ramp up
            s_Shooter.startEnd(() -> 
                s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> 
                s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
            .withTimeout(0.05)
        );

        NamedCommands.registerCommand("Run Shooter",
            s_Shooter.runOnce(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -1.0))
        );

        NamedCommands.registerCommand("Stop Shooter",
            s_Shooter.runOnce(() -> s_Shooter.runShooter(0, 0, 0))
        );

        NamedCommands.registerCommand("Enable Auto Aim",
            Commands.parallel(
                s_Swerve.runOnce(() -> s_Swerve.setAutoRotationOverride(true)),
                s_Shooter.run(() -> s_Shooter.setAngleFromPose(s_Swerve.getPose())).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            )
        );

        NamedCommands.registerCommand("Disable Auto Aim",
            Commands.parallel(
                s_Swerve.runOnce(() -> s_Swerve.setAutoRotationOverride(false)),
                s_Shooter.runOnce(() -> s_Shooter.setAngle(ShooterConstants.kShooterMinAngle, false))
            )
        );

        NamedCommands.registerCommand("Intake Down", 
            s_Intake.runOnce(() -> s_Intake.bottomPosition())
        );

        NamedCommands.registerCommand("Intake Up", 
            s_Intake.runOnce(() -> s_Intake.topPosition())
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