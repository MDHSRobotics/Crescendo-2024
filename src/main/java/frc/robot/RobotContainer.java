package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import frc.math.Aiming;
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
    private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
    private final CommandXboxController operatorController = new CommandXboxController(1); // My joystick

    /* Subsystems */
    private final Swerve s_Swerve = TunerConstants.DriveTrain; // My drivetrain
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Climb s_Climb = new Climb();
    private final LED s_Led = new LED();
    
    DigitalInput intakeLimitSwitch = new DigitalInput(0);
    DigitalInput shooterLimitSwitch = new DigitalInput(1);

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveSpeedConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveSpeedConstants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                // driving in open loop

    // Set up telemetry
    private final Telemetry logger = new Telemetry(SwerveSpeedConstants.MaxSpeed);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Default Commands */
        s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
            s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        if (Utils.isSimulation()) {
        s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        s_Swerve.registerTelemetry(logger::telemeterize);

        s_Led.setDefaultCommand(
            new RunCommand(()-> s_Led.rainbow(), s_Led)
        );
        
        /*s_Shooter.setDefaultCommand(
            new RunCommand(() -> s_Shooter.runShooter(0, 0), s_Shooter)
        );*/

        s_Shooter.setDefaultCommand(
            new RunCommand(()-> s_Shooter.runAngle(operatorController.getLeftY() * 0.5, -operatorController.getRightY() * 0.5), s_Shooter)
        );

        s_Climb.setDefaultCommand(
            new RunCommand((() -> s_Climb.runMotors(0,0)), s_Climb)
        );

        s_Intake.setDefaultCommand(
            new RunCommand(() -> s_Intake.runIntake(0, -operatorController.getRightY()), s_Intake)
        );

        SmartDashboard.putData(s_Shooter);
        SmartDashboard.putData(s_Led);
        SmartDashboard.putData(s_Climb);

        new Trigger(intakeLimitSwitch::get).onTrue(new RunCommand(() -> s_Led.setColor(240, 161, 26), s_Led));

        // Configure the button bindings
        configureButtonBindings();

        /* Named Auto Commands */
        
        NamedCommands.registerCommand("Shoot", 
            new SequentialCommandGroup(
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(1, 0), s_Shooter).withTimeout(2),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(1, 0.2), s_Shooter).withTimeout(2)
            )
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */

        // Reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        driverController.povUp().onTrue(new RunCommand(()-> s_Led.blink(0, 0, 255, 1000), s_Led).withTimeout(10));
        driverController.povDown().onTrue(new RunCommand(()-> s_Led.blink(255, 255, 0, 1000), s_Led).withTimeout(10));
        
        // Climb
        driverController.x().whileTrue(new RunCommand(() -> s_Climb.runMotors(1,0), s_Climb));
        driverController.a().whileTrue(new RunCommand(() -> s_Climb.runMotors(-1, 0), s_Climb));
        driverController.y().whileTrue(new RunCommand(() -> s_Climb.runMotors(0, 1), s_Climb));
        driverController.b().whileTrue(new RunCommand(() -> s_Climb.runMotors(0, -1), s_Climb));

        /* Operator Buttons */

        // Run intake
        operatorController.y().whileTrue(new RunCommand(() -> s_Intake.runIntake(-operatorController.getRightX(), operatorController.getRightTriggerAxis()), s_Intake));

        // Lock on
        operatorController.x().toggleOnTrue(
            s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed) // Drive forward with // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(Aiming.getYawAdjustment(LimelightHelper.getTX(""))) // Turn at the rate given by limelight
            )
            .alongWith(
                new RunCommand(() -> s_Shooter.setAngleFromLimelight(), s_Shooter)
            ).alongWith(
                new RunCommand(() -> s_Led.setColor(255, 0, 0), s_Led)
            )
        );

        
        // Shoot
        operatorController.a().onTrue(
            new SequentialCommandGroup(
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(1, 0), s_Shooter).withTimeout(2),
                
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(1, 0.2), s_Shooter).withTimeout(2)
            )
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0), s_Shooter).withTimeout(2),
                // Blink green to indicate good to go
                new RunCommand(() -> s_Led.setColor(0, 255, 0), s_Led).withTimeout(2)
            )
        );

        // Manual Control
    }

    public static double roundAvoid(double value, int places) {
        double scale = Math.pow(10, places);
        double newValue = Math.round(value * scale) / scale;
        return newValue; 
    }

    /**
     * Use this to pass the a   utonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        System.out.println("In auto");
        return new PathPlannerAuto("Test Auto");
    }
    
}