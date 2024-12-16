package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private DigitalInput m_leftlimitSwitch = new DigitalInput(ClimbConstants.kLeftLimitSwitchID);
  private DigitalInput m_rightlimitSwitch = new DigitalInput(ClimbConstants.kRightLimitSwitchID);

  /* Shuffleboard Logging */
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  // Lowest value is -225
  private GenericEntry leftRotations = tab.add("Left Rotations", 0.0).getEntry();
  // Highest value is 225
  private GenericEntry rightRotations = tab.add("Right Rotations", 0.0).getEntry();

  public Climb() {
    leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
    rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);

    leftClimb.setInverted(true);

    leftEncoder = leftClimb.getEncoder();
    rightEncoder = rightClimb.getEncoder();

    rightClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setIdleMode(IdleMode.kBrake);

    // Right Climb Spark Max was the only controller with too high of a current limit. You can
    // connect to it and change it, but we just put it here for now.
    rightClimb.setSmartCurrentLimit(40);

    // CAN optimization:
    // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
    for (int i = 0; i < 8; ++i) {
      PeriodicFrame frame = PeriodicFrame.fromId(i);
      leftClimb.setPeriodicFramePeriod(frame, 500);
      rightClimb.setPeriodicFramePeriod(frame, 500);
    }
  }

  /**
   * @return True if at least one of the switches is pressed, false if both switches are unpressed
   */
  public boolean getLimitSwitches() {
    return m_leftlimitSwitch.get() || m_rightlimitSwitch.get();
  }

  /* Instance Command Factory Methods
   * These methods allow us to create single-subsystem commands directly in the subsystems, instead of placing them in RobotContainer.
   * https://docs.wpilib.org/en/latest/docs/software/commandbased/organizing-command-based.html#instance-command-factory-methods
   */

  /**
   * This command runs the climb at the specified power until the command is interrupted.
   *
   * <p>Since the command is easily reusable, it's better to name the command in RobotContainer
   * wherever it is used.
   */
  public Command runClimbCommand(double leftPower, double rightPower) {
    return this.runOnce(
            () -> {
              leftClimb.set(leftPower);
              rightClimb.set(rightPower);
            })
        .andThen(Commands.idle(this));
  }

  public void logData() {
    leftRotations.setDouble(leftEncoder.getPosition());
    rightRotations.setDouble(rightEncoder.getPosition());
  }
}
