package frc.robot.subsystems;
 
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.math.Aiming;
import frc.robot.LimelightHelper;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
    @Override
    public void periodic() {
        SmartDashboard.putNumber("tx", LimelightHelper.getTX(""));
        SmartDashboard.putNumber("ty", LimelightHelper.getTY(""));
        double targetDistance = LimelightHelper.getTargetPose3d_CameraSpace("").getTranslation().getDistance(new Translation3d());
        SmartDashboard.putNumber("Apriltag Distance", Aiming.calculateDistance(35.0, 57.0, 17, LimelightHelper.getTY("")));
        SmartDashboard.putNumber("Apriltag Distance 3D", Aiming.calculateDistance3d(targetDistance, targetDistance))
    }
}
