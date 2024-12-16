package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    // When connected to the RoboRio, use this constructor because it will use the
    // proper period duration
    public Robot() {
        super();
    }

    // When running in Simulation mode (not connected to the RoboRio), use this
    // constructor because it can specify a longer period duration which avoids
    // watchdog overruns that can occur since the Simulator uses the VSCode debugger
    public Robot(double period) {
        super(period);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Initialize our RobotManager, which initializes and perists the state of the robot,
        // including flags, sensors, devices, subsystems, commands, shuffleboard,
        // and puts our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Set the log path for SysId
        SignalLogger.setPath("/logs");
        // Start logging data from Swerve and Telemetry.java
        // SignalLogger.start();
        // Start logging NetworkTables (including Shuffleboard)
        DataLogManager.start();
        // Record joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        // Start logging REV Spark Max and Spark Flex data
        // Not working with NetworkTables?
        URCL.start(Constants.sparkDeviceNames);

        // Set the limelight pipeline.
        // In Orange County Regionals, the lights make Apriltags hard to see, so we may change the front
        // to use a separate pipeline.
        // LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontName,
        // DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? 1 : 0);
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontName, 0);

        // Uncomment this if you want to hide "controller not connected" warnings.
        // DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Log the battery voltage going into the roboRIO, in Volts.
        SmartDashboard.putNumber("Battery Voltage (V)", RobotController.getBatteryVoltage());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledExit() {
        // Set the priority tag depending on alliance. This means we will get tx and ty from this tag,
        // but limelight pose measurements from all tags.
        // https://docs.limelightvision.io/docs/docs-limelight/software-change-log#new-feature-priority-id-nt-key-priorityid
        if (DriverStation.getAlliance().orElseThrow() == Alliance.Blue) {
            LimelightHelpers.setPriorityTagID(LimelightConstants.kFrontName, 7);
        } else {
            LimelightHelpers.setPriorityTagID(LimelightConstants.kFrontName, 4);
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link BotCommands} class. */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            System.out.println(m_autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        m_robotContainer.logSubsystemData();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        // Set the starting perspective for driving.
        m_robotContainer.setOperatorPerspective(
                Rotation2d.fromDegrees(DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? 0 : 180));

        SignalLogger.start();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.logSubsystemData();
    }

    @Override
    public void teleopExit() {
        SignalLogger.stop();
        DataLogManager.stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        // Set the starting position to right below the red speaker.
        m_robotContainer.setStartingPosition(new Pose2d(15.15, 5.55, Rotation2d.fromDegrees(180)));

        // Set the starting perspective for driving.
        m_robotContainer.setOperatorPerspective(
                Rotation2d.fromDegrees(DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? 0 : 180));
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        m_robotContainer.logSubsystemData();
    }

    @Override
    public void testExit() {
        SignalLogger.stop();
        DataLogManager.stop();
    }
}
