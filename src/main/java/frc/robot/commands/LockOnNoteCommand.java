package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

/**
 * This command locks on to a note if TX has updated since the command was last called.
 * This check is necessary because note detection runs at 10 fps, which is slower than the robot period.
 * It's useful to update the drving of the robot every frame,
 * but if we updated the target direction using an up-to-date robot angle and an old TX,
 * we would introduce setpoint overshoot into the PID controller.
 * Since this command needs to track the state of TX, it should be created in its own class.
 * This should only be run when a note is detected (when TX != 0). Having a Trigger for this is recommended.
 */
public class LockOnNoteCommand extends Command {
    private final RobotContainer m_robotContainer;
    private final Swerve m_swerve;
    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest;
    private double m_previousTX = 0;

    public LockOnNoteCommand(RobotContainer robotContainer, Swerve swerve, SwerveRequest.FieldCentricFacingAngle angleRequest) {
        m_robotContainer = robotContainer;
        m_swerve = swerve;
        addRequirements(m_swerve);
        m_angleRequest = angleRequest;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // This command should only be run when a note is detected, so that TX is not 0.
        double currentTX = LimelightHelpers.getTX("limelight-back");
        m_angleRequest.VelocityX = m_robotContainer.getVelocityX();
        m_angleRequest.VelocityY = m_robotContainer.getVelocityY();

        // If tx has updated, update the target direction.
        if (currentTX != m_previousTX) {
            m_previousTX = currentTX;

            Rotation2d currentDirection = m_swerve.getPose().getRotation();
            Rotation2d requiredRotation = Rotation2d.fromDegrees(currentTX);
            Rotation2d targetDirection = currentDirection.minus(requiredRotation);
            // If the alliance is red, the driveFacingAngle request will incorrectly try to rotate the target direction, so rotate it back
            if (m_robotContainer.kAlliance == Alliance.Red) {
            targetDirection = targetDirection.rotateBy(Rotation2d.fromDegrees(-180));
            }
            m_angleRequest.TargetDirection = targetDirection;
        }

        m_swerve.setControl(m_angleRequest);
    }

    @Override
    public void end(boolean interrupted) {
        // Reset previous TX.
        m_previousTX = 0;
    }
}