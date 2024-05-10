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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.math.Aiming;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSpeedConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

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
    DigitalInput shooterLimitSwitch = new DigitalInput(0);
    DigitalInput climbLimitSwitch = new DigitalInput(2);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * Constants.stickDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * Constants.stickDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // Set up telemetry. Disabled until we need Swerve module states.
    // private final Telemetry logger = new Telemetry(SwerveSpeedConstants.MaxSpeed);

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Robot Modes */
    private boolean m_slowMode = false;
    private boolean m_isAmp = false;
    private boolean m_autoshoot = false;

    public static Alliance kAlliance;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Default Commands */
        if (driverController != null) {
            s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
                    s_Swerve.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Forward and backward speed
                        .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Left and right speed
                        .withRotationalRate(-driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate * (m_slowMode ? 1.0 : 1.0))) // Rotation speed
            );
        }

        // Disabled until we need Swerve module states.
        // s_Swerve.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        if (operatorController != null) {
            s_Shooter.setDefaultCommand(
                s_Shooter.run(() -> s_Shooter.rotateShooter(operatorController.getRightY()))
            );

            s_Intake.setDefaultCommand(
                s_Intake.run(() -> s_Intake.topPosition(operatorController.getLeftY()))
            );
        }

        s_Led.setDefaultCommand(
            s_Led.run(()-> s_Led.rainbow())
        );

        // LEDs glow orange for 3 secs whenever a note is picked up.
        new Trigger(shooterLimitSwitch::get).onTrue(s_Led.run(() -> s_Led.setColor(255, 20, 0)).withTimeout(3));

        s_Climb.setDefaultCommand(
            s_Climb.runOnce((() -> s_Climb.runClimb(0,0)))
        );


        // Configure the button bindings
        configureButtonBindings();


        /* Named Auto Commands */
        NamedCommands.registerCommand("Start Shooter",  
            // Tuck the note into the shooter, and then ramp up
            s_Shooter.startEnd(() -> 
                s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> 
                s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
            .withTimeout(0.05)
        );

        NamedCommands.registerCommand("Shoot",
            s_Shooter.runOnce(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -1.0))
        );

        NamedCommands.registerCommand("Auto Aim",
            s_Swerve.runOnce(() -> s_Swerve.setAutoRotationOverride(true))
            .andThen(s_Shooter.run(() -> s_Shooter.setAngle(Aiming.getPitch(s_Swerve.getPose()))))
        );

        NamedCommands.registerCommand("Stop Shooter", 
            s_Shooter.runOnce(() -> s_Shooter.runShooter(0.0, 0.0, 0.0))
        );

        NamedCommands.registerCommand("Intake Down", 
            s_Intake.runOnce(() -> s_Intake.bottomPosition())
        );

        NamedCommands.registerCommand("Intake Up", 
            s_Intake.runOnce(() -> s_Intake.topPosition(0))
        );
        

        /* Add Subsystem Sendable tables to Shuffleboard */
        Shuffleboard.getTab("Swerve").add("Kinematics + Odometry", s_Swerve).withSize(2, 2);
        Shuffleboard.getTab("Shooter").add("Shooter Info", s_Shooter).withSize(3, 3);
        Shuffleboard.getTab("Climb").add("Climb Info", s_Climb).withSize(3, 2);
        Shuffleboard.getTab("Intake").add("Intake Info", s_Intake).withSize(3, 2);

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
        https://www.padcrafter.com/index.php?templates=Driver+Controller&leftBumper=Climb+Down&dpadRight=&dpadLeft=&aButton=Brake+%28cross+wheels%29&yButton=Right+Climb+Up&dpadDown=Tell+Human+Player+to+Amplify&dpadUp=Tell+Human+Player+to+Coopertition&xButton=Left+Climb+Up&bButton=&leftStick=Drive+Robot+in+this+Direction&rightStick=Rotate+Robot&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&rightTrigger=Fast+Mode&leftTrigger=Slow+Mode&rightBumper=Climb+Up&startButton=Reset+Field+Oriented+Drive&plat=1&backButton=
        Whenever you edit a button binding, please update this URL
        */

        // Brake the robot by crossing the wheels.
        // If the operator locks onto the speaker, the wheels will stop braking.
        driverController.cross().whileTrue(s_Swerve.applyRequest(() -> brake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Reset the field-centric heading on left bumper press. For pose estimation to start working again, drive to an Apriltag.
        driverController.options().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        driverController.povUp().onTrue(s_Led.run(() -> s_Led.blink(0, 0, 255, 400)).withTimeout(5)); // Coopertition
        driverController.povDown().onTrue(s_Led.run(() -> s_Led.blink(255, 255, 0, 1000)).withTimeout(5)); // Amplify
        
        // Climb
        driverController.L1().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(-1, -1), () -> {}).until(climbLimitSwitch::get));
        driverController.R1().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 1), () -> {}));
        driverController.square().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(1, 0), () -> {}));
        driverController.povRight().whileTrue(s_Climb.startEnd(() -> s_Climb.runClimb(0, 1), () -> {}));

        driverController.R2().onTrue(s_Swerve.runOnce(() -> m_slowMode = false));
        driverController.L2().onTrue(s_Swerve.runOnce(() -> m_slowMode = true));
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the operator controller:
            https://www.padcrafter.com/?rightStickClick=Lock+Speaker+%28using+pose%29&xButton=Lock+Speaker&aButton=Fire&bButton=Set+Angle%3A+Amp&rightStick=Aim+Shooter+%28Calibration+Only%29&rightBumper=Set+Angle%3A+Point+Blank&rightTrigger=Deploy+Intake&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&leftStick=Aim+Intake+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadLeft=Calibration+Mode+Toggle&dpadDown=Reset+Intake+Encoder&startButton=Free+a+Stuck+Note+%28on+shooter%29&backButton=Eject+Intake&templates=Operator+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF#?xButton=Lock+Speaker&leftTrigger=Deploy+Intake+(Slightly+Above+Ground)&leftBumper=Set+Angle%3A+Podium&rightBumper=Set+Angle%3A+Point+Blank&leftStick=Aim+Intake+(Calibration+Only)&rightStick=Aim+Shooter+(Calibration+Only)&dpadUp=Reset+Shooter+Encoder&dpadDown=Reset+Intake+Encoder&dpadLeft=Calibration+Mode+Toggle&rightTrigger=Deploy+Intake&backButton=Eject+Intake&bButton=Set+Angle%3A+Amp&aButton=Fire&yButton=Pass+Note+(Straight+Shot)&startButton=Free+a+Stuck+Note+(on+shooter)&templates=Controller+Scheme+1
            Please update this link whenever you change a button.
        */
        
        // Run intake
        operatorController.rightTrigger().toggleOnTrue(
            s_Intake.startEnd(() -> s_Intake.bottomPosition(), () -> {})
            .alongWith(s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.7), () -> {})));

        operatorController.leftTrigger().toggleOnTrue(
            s_Intake.startEnd(() -> s_Intake.midPosition(), () -> {})
            .alongWith(s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, -0.7), () -> {})));

        operatorController.back().toggleOnTrue(
            s_Intake.startEnd(() -> s_Intake.spitOut(), () -> {})
            .alongWith(s_Shooter.startEnd(() -> s_Shooter.runShooter(0, 0, 1.0), () -> {})));


        // Lock on to speaker (using limelight)
        operatorController.x()
        .toggleOnTrue(
            Commands.sequence(
                // Tuck note into shooter and then ramp up
                s_Shooter.startEnd(() -> 
                    s_Shooter.runShooter(-0.2, -0.2, 0.5), () ->
                    s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, 0))
                    .withTimeout(0.05),

                Commands.parallel(
                    s_Swerve.applyRequest(() -> driveFacingAngle
                        .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with // negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                        .withTargetDirection(Rotation2d.fromDegrees(s_Swerve.getAngle() - LimelightHelpers.getTX("")))),
                        
                    s_Shooter.run(() -> s_Shooter.setAngleFromLimelight())
                )
            ).until(() -> Math.abs(driverController.getRightX()) > Constants.stickDeadband)
        );
        // When it locks on speaker, set shoot mode to speaker instead of amp
        operatorController.x().onTrue(s_Shooter.runOnce(() -> m_isAmp = false));
        
        // Lock on to speaker (using pose estimation)
        operatorController.rightStick()
        .toggleOnTrue(
            Commands.parallel(
                s_Swerve.applyRequest(() -> driveFacingAngle
                    .withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with // negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                    .withTargetDirection(Aiming.getYaw(s_Swerve.getPose()))),
                        
                s_Shooter.run(() -> s_Shooter.setAngle(Aiming.getPitch(s_Swerve.getPose())))
            ).until(() -> Math.abs(driverController.getRightX()) > 0.1)
        );
        // When it locks on speaker, set shoot mode to speaker instead of amp
        operatorController.rightStick().onTrue(s_Shooter.runOnce(() -> m_isAmp = false));

        // Set angle to amp
        operatorController.b()
        .toggleOnTrue(
                s_Shooter.startEnd(() -> s_Shooter.setAngle(52.0), () -> {})
            .alongWith(
                s_Led.startEnd(() -> s_Led.setColor(255, 0, 0), () -> {})
            )
        );
        // When it sets the angle to amp, set shoot mode to amp as well
        operatorController.b().onTrue(s_Shooter.runOnce(() -> m_isAmp = true));
       

        // Shoot Speaker
        operatorController.a()
        .and(new Trigger(() -> !m_isAmp))
        .onTrue(
            // Shoot and then stop
            s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), () -> 
            s_Shooter.runShooter(0, 0, 0))
            .withTimeout(0.5)
        );

        new Trigger(s_Shooter::isReady)
            .and(() -> m_autoshoot)
            .onTrue(
                s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), () -> 
                s_Shooter.runShooter(0, 0, 0))
                .withTimeout(0.5)
        );

        new Trigger(s_Shooter::isReady)
            .onTrue(
                s_Led.startEnd(() -> s_Led.setColor(0, 255, 0), () -> {})
                .until(() -> !s_Shooter.isReady())
            );

        new Trigger(() -> s_Shooter.tagInSight() && !s_Shooter.isReady())
            .onTrue(
                s_Led.run(() -> s_Led.blink(255, 0, 0, 300))
                .until(() -> !s_Shooter.tagInSight())
            );

        // Shoot Amp
        operatorController.a()
            .and(new Trigger(() -> m_isAmp))
            .onTrue(
            Commands.sequence(
                // Tuck note into shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {}).withTimeout(0.05),
                // Ramp up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, 0), () -> {}).withTimeout(1.0),
                // Shoot and then stop shooting
                s_Shooter.startEnd(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, -0.5), () ->
                s_Shooter.runShooter(0, 0, 0))
                .withTimeout(0.2)
            )
        );

        // Calibration Mode
        operatorController.povLeft().onTrue(
            s_Shooter.runOnce(() -> s_Shooter.setCalibration())
            .alongWith(
                s_Intake.runOnce(() -> s_Intake.setCalibration())
            )
        );
        // Recommended to only use this in calibration mode
        operatorController.povUp().onTrue(s_Shooter.runOnce(() -> s_Shooter.resetEncoders()));
        operatorController.povDown().onTrue(s_Intake.runOnce(() -> s_Intake.resetEncoders()));

        // Manual Angle Modes
        operatorController.y().onTrue(
            Commands.sequence(
                // Tuck note into shooter
                s_Shooter.startEnd(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), () -> {}).withTimeout(0.05),
                // Ramp up
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, 0), () -> {}).withTimeout(1.0),
                // Shoot
                s_Shooter.startEnd(() -> s_Shooter.runShooter(0.5, 0.5, -0.5), () ->
                    s_Shooter.runShooter(0, 0, 0))
                    .withTimeout(0.5)
            )
        );

        operatorController.rightBumper()
            .onTrue(s_Shooter.runOnce(() -> m_isAmp = false)
            .andThen(s_Shooter.startEnd(() -> s_Shooter.setAngle(51), () -> {})));

        operatorController.leftBumper()
            .onTrue(s_Shooter.runOnce(() -> m_isAmp = false)
            .andThen(s_Shooter.startEnd(() -> s_Shooter.setAngle(36), () -> {})));


        // Free a stuck note on the top of the robot
        operatorController.start().whileTrue(s_Shooter.startEnd(() -> s_Shooter.runShooter(-1,-1,0), () -> 
            s_Shooter.runShooter(0, 0, 0)));
    }

    public static double roundAvoid(double value, int places) {
        double scale = Math.pow(10, places);
        double newValue = Math.round(value * scale) / scale;
        return newValue; 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setStartingPosition(Pose2d startingPosition){
        s_Swerve.seedFieldRelative(startingPosition);
    }

    public void logData() {
        s_Swerve.updatePose();
        s_Shooter.logData();
    }
}