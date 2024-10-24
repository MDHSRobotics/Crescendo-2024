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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public final Swerve s_Swerve = TunerConstants.DriveTrain; // My drivetrain
    public final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Climb s_Climb = new Climb();
    private final LED s_Led = new LED();
    
    /* Limit Switches */
    DigitalInput shooterLimitSwitch = new DigitalInput(0);
    DigitalInput climbLimitSwitch = new DigitalInput(2);

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

    private boolean m_isAmp = false;
    private boolean m_autoshoot = false;

    public static Alliance kAlliance;

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
            new RunCommand(()-> s_Led.rainbow(), s_Led)
        );

        s_Climb.setDefaultCommand(
            new RunCommand((() -> s_Climb.runClimb(0,0)), s_Climb)
        );

        SmartDashboard.putData(s_Shooter);
        SmartDashboard.putData(s_Led);
        //SmartDashboard.putData(s_Climb);
        SmartDashboard.putData(s_Intake);

        new Trigger(shooterLimitSwitch::get).onTrue(new RunCommand(() -> s_Led.setColor(255, 20, 0), s_Led).withTimeout(3));

        // Configure the button bindings
        configureButtonBindings();

        /* Named Auto Commands */
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
                new RunCommand(() -> s_Shooter.setAngle(40), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 30", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(30), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 32", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(32), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 35", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(35), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 37", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(37), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 45", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(45), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 51", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(51), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 42", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(42), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Angle 40", 
                //Shoot
                new RunCommand(() -> s_Shooter.setAngle(40), s_Shooter).withTimeout(0.5)
        );

        NamedCommands.registerCommand("Feeder On", 
                //Shoot
                new RunCommand(() -> s_Shooter.runFeed(1), s_Shooter).withTimeout(0.5)
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
                new RunCommand(() -> s_Intake.topPosition(0), s_Intake).withTimeout(0.5)
        );

        

        /* Auto Chooser */
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        Shuffleboard.getTab("Main").add(autoChooser).withSize(2, 1);
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

        // IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
        //           trigger what commands on the driver controller:
        //
        // https://www.padcrafter.com/index.php?templates=Driver+Controller&leftBumper=Reset+field+oriented+drive&dpadRight=Right+Climb+Up&dpadLeft=Left+Climb+Up&aButton=&yButton=&dpadDown=Climb+Down&dpadUp=Climb+Up&xButton=&bButton=&leftStick=Field+oriented+drive+direction%2Fvelocity&rightStick=Robot+rotation&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=Fast+mode&leftTrigger=Slow+mode&plat=1

        //
        // Whenever you edit a button binding, please update this URL

        // Reset the field-centric heading on left bumper press
        driverController.L1().onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));

        // LED communication
        //driverController.R1().onTrue(new RunCommand(()-> s_Led.blink(0, 0, 255, 400), s_Led).withTimeout(5));
        //driverController.povRight().onTrue(new RunCommand(()-> s_Led.blink(255, 255, 0, 1000), s_Led).withTimeout(5));
        
        // Climb
        driverController.povLeft().whileTrue(new RunCommand(() -> s_Climb.runClimb(0,1), s_Climb));
        driverController.povDown().whileTrue(new RunCommand(() -> s_Climb.runClimb(-1, -1), s_Climb).until(climbLimitSwitch::get));
        driverController.povUp().whileTrue(new RunCommand(() -> s_Climb.runClimb(1, 1), s_Climb));
        driverController.povRight().whileTrue(new RunCommand(() -> s_Climb.runClimb(1, 0), s_Climb));

        driverController.R2().onTrue(new InstantCommand(() -> m_slowMode = false));
        driverController.L2().onTrue(new InstantCommand(() -> m_slowMode = true));

        //driverController.x().toggleOnTrue(new RunCommand(() -> s_Shooter.runShooter(0.6, -1), s_Shooter));
        //driverController.b().toggleOnFalse(new RunCommand(() -> s_Intake.bottomPosition(), s_Intake));
    }

    private void configureOperatorButtonBindings() {

        /* Operator Buttons */

        /* IMPORTANT Please see the following URL to get a graphical annotation of which xbox buttons 
            trigger what commands on the operator controller:
            https://www.padcrafter.com/?xButton=Lock+Speaker&leftTrigger=Deploy+Intake+%28Slightly+Above+Ground%29&leftBumper=Set+Angle%3A+Podium&rightBumper=Set+Angle%3A+Point+Blank&leftStick=Aim+Intake+%28Calibration+Only%29&rightStick=Aim+Shooter+%28Calibration+Only%29&dpadUp=Reset+Shooter+Encoder&dpadDown=Reset+Intake+Encoder&dpadLeft=Calibration+Mode+Toggle&rightTrigger=Deploy+Intake&backButton=Eject+Intake&bButton=Set+Angle%3A+Amp&aButton=Fire&yButton=Pass+Note+%28Straight+Shot%29&startButton=Free+a+Stuck+Note+%28on+shooter%29#
            Please update this link whenever you change a button.
        */
        
        // Run intake
        operatorController.rightTrigger().toggleOnTrue(new RunCommand(() -> s_Intake.bottomPosition(), s_Intake).alongWith(new RunCommand(() -> s_Shooter.runFeed(0.6), s_Shooter)));
        operatorController.leftTrigger().toggleOnTrue(new RunCommand(() -> s_Intake.midPosition(), s_Intake).alongWith(new RunCommand(() -> s_Shooter.runFeed(0.6), s_Shooter)));
        operatorController.back().toggleOnTrue(new RunCommand(() -> s_Intake.spitOut(), s_Intake).alongWith(new RunCommand(() -> s_Shooter.runFeed(-1.0), s_Shooter)));

        // Lock on to speaker
        operatorController.x()
            .toggleOnTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), s_Shooter).withTimeout(0.05),

                new ParallelCommandGroup(
                    s_Swerve.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * SwerveSpeedConstants.MaxSpeed) // Drive forward with // negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * SwerveSpeedConstants.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(Aiming.getYawTxAdjustment(LimelightHelpers.getTX("limelight-front")))), // Turn at the rate given by limelight
                        
                    new RunCommand(() -> s_Shooter.setAngleFromLimelight(), s_Shooter)
                )
            )
            .until(
                () -> Math.abs(driverController.getRightX()) > 0.1
            )
        );
        operatorController.x().onTrue(new InstantCommand(() -> m_isAmp = false));
        
        // Lock on to amp
        operatorController.b()
            .toggleOnTrue(
                new RunCommand(() -> s_Shooter.setAngle(52.0), s_Shooter)
            .alongWith(
                new RunCommand(() -> s_Led.setColor(255, 0, 0), s_Led)
            )
        );
        operatorController.b().onTrue(new InstantCommand(() -> m_isAmp = true));
       

        // Shoot Speaker
        operatorController.a()
            .and(new Trigger(() -> !m_isAmp))
            .onTrue(
            new RunCommand(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), s_Shooter).withTimeout(0.5)
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0, 0), s_Shooter).withTimeout(0.1)
            )
        );

        new Trigger(s_Shooter::isReady)
            .and(() -> m_autoshoot)
            .onTrue(
                new RunCommand(() -> s_Shooter.runShooter(ShooterConstants.speakerSpeed, ShooterConstants.speakerSpeed, -0.5), s_Shooter).withTimeout(0.5)
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0, 0), s_Shooter).withTimeout(0.1)
            )
        );

        new Trigger(s_Shooter::isReady).onTrue(new RunCommand(() -> s_Led.setColor(0, 255, 0), s_Led).until(() -> !s_Shooter.isReady()));
        new Trigger(() -> s_Shooter.tagInSight() && !s_Shooter.isReady()).onTrue(new RunCommand(() -> s_Led.blink(255, 0, 0, 300), s_Led).until(() -> !s_Shooter.tagInSight()));

        // Shoot Amp
        operatorController.a()
            .and(new Trigger(() -> m_isAmp))
            .onTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, 0), s_Shooter).withTimeout(1.0),
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(ShooterConstants.ampTopSpeed, ShooterConstants.ampBottomSpeed, -0.5), s_Shooter).withTimeout(0.2)
            )
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0, 0), s_Shooter).withTimeout(1)
                // Blink green to indicate good to go
                //new RunCommand(() -> s_Led.setColor(0, 255, 0), s_Led).withTimeout(2)
            )
        );

        // Calibration Mode
        operatorController.povLeft().onTrue(
            new InstantCommand(() -> s_Shooter.setCalibration(), s_Shooter)
            .alongWith(
                new InstantCommand(() -> s_Intake.setCalibration(), s_Intake)
            )
        );
        operatorController.povUp().onTrue(new InstantCommand(() -> s_Shooter.resetEncoders(), s_Shooter));
        operatorController.povDown().onTrue(new InstantCommand(() -> s_Intake.resetEncoders(), s_Intake));

        // Manual Angle Modes
        operatorController.y().onTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> s_Shooter.runShooter(-0.2, -0.2, 0.5), s_Shooter).withTimeout(0.05),
                // Ramp up
                new RunCommand(() -> s_Shooter.runShooter(0.5, 0.5, 0), s_Shooter).withTimeout(1.0),
                //Shoot
                new RunCommand(() -> s_Shooter.runShooter(0.5, 0.5, -0.5), s_Shooter).withTimeout(0.5)
            )
            .andThen(
                new RunCommand(() -> s_Shooter.runShooter(0, 0, 0), s_Shooter).withTimeout(0.5)
            )
        );

        operatorController.rightBumper().onTrue(new RunCommand(() -> s_Shooter.setAngle(51), s_Shooter));
        operatorController.leftBumper().onTrue(new RunCommand(() -> s_Shooter.setAngle(36), s_Shooter));

        operatorController.start().toggleOnTrue(new RunCommand(() -> s_Shooter.runShooter(-1,-1,1), s_Shooter));
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