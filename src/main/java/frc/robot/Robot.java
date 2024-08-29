package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.math.Aiming;
import frc.robot.Constants.PoseConstants;

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
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        // Stop logging for SysId
        SignalLogger.stop();

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link BotCommands} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        CommandScheduler.getInstance().cancelAll();

        // Schedule the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            System.out.println(m_autonomousCommand);
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        m_robotContainer.logSubsystemData();
    }

    @Override
    public void teleopInit() {
        // Use a different limelight pipeline depending on alliance.
        // Only necessary in Orange County Regionals, where the lights make Apriltags hard to see.
        // RobotContainer.kAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        // LimelightHelpers.setPipelineIndex("", RobotContainer.kAlliance == Alliance.Blue ? 1 : 0);

        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.logSubsystemData();
    }

    @Override
    public void testInit() {
        // Use a different limelight pipeline depending on alliance.
        // Only necessary in Orange County Regionals, where the lights make Apriltags hard to see.
        // RobotContainer.kAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        // LimelightHelpers.setPipelineIndex("", RobotContainer.kAlliance == Alliance.Blue ? 1 : 0);

        // Sets a starting pose for pose estimation to right below the Speaker. Open this project in PathPlanner for more starting positions.
        m_robotContainer.setStartingPosition(new Pose2d(0.48, 4.09, new Rotation2d()));

        CommandScheduler.getInstance().cancelAll();

        // Start logging for SysId
        SignalLogger.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        m_robotContainer.logSubsystemData();
    }

}
