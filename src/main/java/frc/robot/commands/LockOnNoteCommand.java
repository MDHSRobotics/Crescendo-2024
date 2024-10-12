package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.SwerveSpeedConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/**
 * This command is meant to lock on to a note if a note is detected,
 * and if TX has updated since the command was last called.
 * The latter check is necessary because note detection runs at 10 fps, which is slower than the robot period.
 * It's useful to update the drving of the robot every frame,
 * but if we updated the target direction using an up-to-date robot angle and an old TX,
 * we would introduce setpoint overshoot into the PID controller.
 * Since this command needs to track the state of TX, it should be created in its own class.
 */
public class LockOnNoteCommand extends Command {
    private final Swerve m_swerve;
    private final SwerveRequest.FieldCentric m_defaultDriveRequest;
    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest;
    private final CommandPS4Controller m_driverController;
    private double m_previousTX;

    public LockOnNoteCommand(Swerve swerve, SwerveRequest.FieldCentric defaultDriveRequest, SwerveRequest.FieldCentricFacingAngle angleRequest, CommandPS4Controller driverController) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        m_defaultDriveRequest = defaultDriveRequest;
        m_angleRequest = angleRequest;
        m_driverController = driverController;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the velocity for each request
        double Y = -m_driverController.getLeftY();
        double X = -m_driverController.getLeftX();
        
        // If the limelight can't detect any notes, drive slowly with regular controls.
        double currentTX = LimelightHelpers.getTX("limelight-back");
        if (currentTX == 0) {
            m_defaultDriveRequest.VelocityX = Y * SwerveSpeedConstants.MaxSpeed * 0.5;
            m_defaultDriveRequest.VelocityY = X * SwerveSpeedConstants.MaxSpeed * 0.5;
            m_defaultDriveRequest.RotationalRate = -m_driverController.getRightX() * SwerveSpeedConstants.MaxAngularRate;

            m_swerve.applyRequest(() -> m_defaultDriveRequest);
        } else { // Else, drive slowly while facing the note.
            m_angleRequest.VelocityX = Y * SwerveSpeedConstants.MaxSpeed * 0.5;
            m_angleRequest.VelocityY = X * SwerveSpeedConstants.MaxSpeed * 0.5;

            // If tx has updated, update the target direction.
            if (currentTX != m_previousTX) {
                m_previousTX = currentTX;

                Rotation2d targetDirection = Rotation2d.fromDegrees(m_swerve.getRobotYaw() - currentTX);
                m_angleRequest.TargetDirection = targetDirection;
            }

            m_swerve.applyRequest(() -> m_angleRequest);
        }
    }
}