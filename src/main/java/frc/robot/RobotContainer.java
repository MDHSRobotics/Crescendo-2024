package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.math.Aiming;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSpeedConstants;
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
    private final CommandXboxController driverController = new CommandXboxController(0); 
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* Subsystems */
    public final Swerve s_Swerve = TunerConstants.DriveTrain; // My drivetrain
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Climb s_Climb = new Climb();
    private final LED s_Led = new LED();
    
    /* Limit Switches */
    DigitalInput intakeLimitSwitch = new DigitalInput(1);
    DigitalInput shooterLimitSwitch = new DigitalInput(0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                // driving in open loop

    // Set up telemetry
    private final Telemetry logger = new Telemetry(SwerveSpeedConstants.MaxSpeed);

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /* Swerve Speeds */
    private boolean m_slowMode = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Default Commands */
        if (driverController != null) {
            s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
                s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive forward with
                                                                                                // negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed * (m_slowMode ? 0.2 : 1.0)) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate * (m_slowMode ? 1.0 : 1.0)) // Drive counterclockwise with negative X (left)
                ));
        }

        if (operatorController != null) {
            s_Shooter.setDefaultCommand(
                new RunCommand(()-> s_Shooter.run(operatorController.getRightY()), s_Shooter)
            );

            s_Intake.setDefaultCommand(
                new RunCommand(() -> s_Intake.topPosition(operatorController.getLeftY()), s_Intake)
            );
        }

        if (Utils.isSimulation()) {
            s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
            s_Swerve.registerTelemetry(logger::telemeterize);

            s_Led.setDefaultCommand(
                new RunCommand(()-> s_Led.setColor(100, 250, 100), s_Led)
        );

        s_Climb.setDefaultCommand(
            new RunCommand((() -> s_Climb.runClimb(0,0)), s_Climb)
        );

        SmartDashboard.putData(s_Shooter);
        SmartDashboard.putData(s_Led);
        SmartDashboard.putData(s_Climb);
        SmartDashboard.putData(s_Intake);

        new Trigger(intakeLimitSwitch::get).onTrue(new RunCommand(() -> s_Led.setColor(240, 161, 26), s_Led));

        // Configure the button bindings
        configureButtonBindings();

        /* Named Auto Commands */
        NamedCommands.registerCommand("Shoot", 
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.7, -1), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Feeder", 
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0, -1), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Shoot Start",  
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(0.7, 0), s_Shooter).withTimeout(1.0),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.7, -0.5), s_Shooter).withTimeout(1)
            )
        );

        NamedCommands.registerCommand("Angle", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(42), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 2", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(30), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Shoot Stop", 
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.0, 0.0), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Intake Down", 
                new RunCommand(() -> s_Intake.bottomPosition(), s_Intake).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Intake Up", 
                //Shoot
                new RunCommand(() -> s_Intake.topPosition(0), s_Intake).withTimeout(0.5)
        );

        /* Auto Chooser */
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
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

        // Reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        driverController.povUp().onTrue(new RunCommand(()-> s_Led.blink(0, 0, 255, 1000), s_Led).withTimeout(10));
        driverController.povDown().onTrue(new RunCommand(()-> s_Led.blink(255, 255, 0, 1000), s_Led).withTimeout(10));
        
        // Climb
        //driverController.x().whileTrue(new RunCommand(() -> s_Climb.runClimb(0,1), s_Climb));
        driverController.a().whileTrue(new RunCommand(() -> s_Climb.runClimb(-1, -1), s_Climb));
        driverController.y().whileTrue(new RunCommand(() -> s_Climb.runClimb(1, 1), s_Climb));
        //driverController.b().whileTrue(new RunCommand(() -> s_Climb.runClimb(1, 0), s_Climb));

        driverController.povUp().onTrue(new InstantCommand(() -> m_slowMode = false));
        driverController.povDown().onTrue(new InstantCommand(() -> m_slowMode = true));

        driverController.x().toggleOnTrue(new RunCommand(() -> s_Shooter.runShooter(0.6, -1), s_Shooter));
        driverController.b().toggleOnFalse(new RunCommand(() -> s_Intake.bottomPosition(), s_Intake));
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        Trigger speakerTag = new Trigger(() -> LimelightHelper.getFiducialID("") == 4 || LimelightHelper.getFiducialID("") == 7);
        Trigger ampTag = new Trigger(() -> LimelightHelper.getFiducialID("") == 5 || LimelightHelper.getFiducialID("") == 6);

        // Run intake
        operatorController.y().toggleOnTrue(new RunCommand(() -> s_Intake.bottomPosition(), s_Intake).alongWith(new RunCommand(() -> s_Shooter.runFeed(0.4), s_Shooter)));
        operatorController.povDown().toggleOnTrue(new RunCommand(() -> s_Intake.spitOut(), s_Intake).alongWith(new RunCommand(() -> s_Shooter.runFeed(-1.0), s_Shooter)));

        // Lock on to speaker
        operatorController.x()
            .and(speakerTag)
            .toggleOnTrue(
            s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed) // Drive forward with // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(Aiming.getYawTxAdjustment(LimelightHelper.getTX(""))) // Turn at the rate given by limelight
            )
            .alongWith(
                new RunCommand(() -> s_Shooter.setAngleFromLimelight(), s_Shooter)
            ).alongWith(
                new RunCommand(() -> s_Led.setColor(255, 0, 0), s_Led)
            ).until(
                () -> Math.abs(driverController.getRightX()) > 0.1
            )
        );

        // Lock on to amp
        operatorController.x()
            .and(ampTag)
            .toggleOnTrue(
            s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed) // Drive forward with // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(Aiming.getYawTxAdjustment(LimelightHelper.getTX(""))) // Turn at the rate given by limelight
            )
            .alongWith(
                new RunCommand(() -> s_Shooter.setAngle(ShooterConstants.ampAngle), s_Shooter)
            ).alongWith(
                new RunCommand(() -> s_Led.setColor(255, 0, 0), s_Led)
            ).until(
                () -> Math.abs(driverController.getRightX()) > 0.1
            )
        );

        operatorController.povLeft().onTrue(
            new InstantCommand(() -> s_Shooter.setCalibration(), s_Shooter)
            .alongWith(
                new InstantCommand(() -> s_Intake.setCalibration(), s_Intake)
            )
        );
        
        // Shoot
        operatorController.a().onTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(0.7, 0), s_Shooter).withTimeout(1.5),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.7, -0.5), s_Shooter).withTimeout(1)
            )
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0), s_Shooter).withTimeout(1)
                // Blink green to indicate good to go
                //new RunCommand(() -> s_Led.setColor(0, 255, 0), s_Led).withTimeout(2)
            )
        );

        operatorController.b().onTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(0.145, 0), s_Shooter).withTimeout(1.5),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.145, -0.5), s_Shooter).withTimeout(1)
            )
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0), s_Shooter).withTimeout(1)
                // Blink green to indicate good to go
                //new RunCommand(() -> s_Led.setColor(0, 255, 0), s_Led).withTimeout(2)
            )
        );

        operatorController.start().onTrue(new InstantCommand(() -> s_Shooter.resetEncoders(), s_Shooter));
        operatorController.back().onTrue(new InstantCommand(() -> s_Intake.resetEncoders(), s_Intake));
        // Manual Control
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
}