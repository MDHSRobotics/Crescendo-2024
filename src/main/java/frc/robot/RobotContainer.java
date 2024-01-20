package frc.robot;

import com.fasterxml.jackson.core.util.RequestPayload;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton shootStop = new JoystickButton(driver, XboxController.Button.kB.value);


    /* Subsystems */
    //private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();

    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /*s_Shooter.setDefaultCommand(
            new RunCommand((() -> s_Shooter.runMotors(translationAxis)), s_Shooter)
        );*/

        //xboxController.a().onTrue(s_Shooter.runMotorsCommand());

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
        shoot.toggleOnTrue(new RunCommand(() -> s_Shooter.runMotors(roundAvoid(xboxController.getLeftY(),1))));
        shootStop.onTrue(new InstantCommand(() -> s_Shooter.runMotors(0)));
    }

    public static double roundAvoid(double value, int places) {
        double scale = Math.pow(10, places);
        double newValue = Math.round(value * scale) / scale;
        newValue = newValue > 0.8 ? 0.8 : newValue;
        newValue = newValue < 0.2 ? 0.2 : newValue;
        return newValue; 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    //public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
    //    return new new Command
    //}
}