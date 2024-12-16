package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    public enum IntakePositions {
        BOTTOM,
        MID,
    }

    private CANSparkFlex intake;
    private CANSparkMax conveyor;
    private CANSparkMax leftAngle;
    private CANSparkMax rightAngle;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_leftAngleEncoder;
    private RelativeEncoder m_rightAngleEncoder;

    /* Shuffleboard Logging */
    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    private ShuffleboardLayout list =
            tab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3, 4);
    private GenericEntry leftAngleRotations =
            list.add("Left Angle Rotations", 0.0).getEntry();
    private GenericEntry rightAngleRotations =
            list.add("Right Angle Rotations", 0.0).getEntry();
    private GenericEntry intakeSpeed = list.add("Intake Speed", 0.0).getEntry();
    private GenericEntry conveyorSpeed = list.add("Conveyer Speed", 0.0).getEntry();

    private GenericEntry intakeTopRotations =
            tab.addPersistent("Top Rotations", 0.0).withSize(2, 1).getEntry();
    private GenericEntry intakeBottomRotations =
            tab.addPersistent("Bottom Rotations", 34.0).withSize(2, 1).getEntry();
    private GenericEntry intakeEjectRotations =
            tab.addPersistent("Eject Rotations", 30.0).withSize(2, 1).getEntry();

    public Intake() {
        intake = new CANSparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
        conveyor = new CANSparkMax(IntakeConstants.kConveyorID, MotorType.kBrushless);
        leftAngle = new CANSparkMax(IntakeConstants.kLeftAngleID, MotorType.kBrushless);
        rightAngle = new CANSparkMax(IntakeConstants.kRightAngleID, MotorType.kBrushless);

        m_leftAngleEncoder = leftAngle.getEncoder();
        m_rightAngleEncoder = rightAngle.getEncoder();

        rightAngle.setIdleMode(IdleMode.kBrake);
        leftAngle.setIdleMode(IdleMode.kBrake);

        m_pidController = rightAngle.getPIDController();
        m_pidController.setP(0.08);

        // CAN optimization:
        // https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        conveyor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        conveyor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        rightAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        rightAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);

        leftAngle.setOpenLoopRampRate(0.1);
        rightAngle.setOpenLoopRampRate(0.1);
        intake.setOpenLoopRampRate(0.1);
        conveyor.setOpenLoopRampRate(0.1);

        leftAngle.setSmartCurrentLimit(40);
        rightAngle.setSmartCurrentLimit(40);

        leftAngle.follow(rightAngle, true);

        conveyor.setInverted(true);
    }

    /* Instance Command Factory Methods
     * These methods allow us to create single-subsystem commands directly in the subsystems, instead of placing them in RobotContainer.
     * https://docs.wpilib.org/en/latest/docs/software/commandbased/organizing-command-based.html#instance-command-factory-methods
     */

    /** This command turns off the rollers and raises the intake to its top position. */
    public Command disableIntakeCommand(boolean returnToTopPosition) {
        Command disableIntakeCommand;
        if (returnToTopPosition) {
            disableIntakeCommand = this.runOnce(() -> {
                intake.set(0);
                conveyor.set(0);
                m_pidController.setReference(intakeTopRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
            });
        } else {
            disableIntakeCommand = this.runOnce(() -> {
                intake.set(0);
                conveyor.set(0);
            });
        }

        disableIntakeCommand = disableIntakeCommand.andThen(Commands.idle(this)).withName("Disable Intake");

        return disableIntakeCommand;
    }

    /** This command enables the rollers and raises the intake to the desired position. */
    public Command intakeNoteCommand(IntakePositions position) {
        Command intakeNoteCommand;
        switch (position) {
            case BOTTOM:
                intakeNoteCommand = this.runOnce(() -> {
                    intake.set(1);
                    conveyor.set(1);
                    m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
                });
                break;
            default: // MID:
                intakeNoteCommand = this.runOnce(() -> {
                    intake.set(1);
                    conveyor.set(1);
                    m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
                });
        }

        intakeNoteCommand = intakeNoteCommand.andThen(Commands.idle(this));

        return intakeNoteCommand;
    }

    /**
     * This command runs the rollers in reverse and raises the intake to the eject position (about
     * halfway). This is occasionally needed when the note gets stuck.
     */
    public Command ejectNoteCommand() {
        return this.runOnce(() -> {
                    intake.set(-1);
                    conveyor.set(-1);
                    m_pidController.setReference(intakeEjectRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
                })
                .andThen(Commands.idle(this));
    }

    /**
     * Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is
     * disabled.
     */
    public void logData() {
        // Subsystem data
        leftAngleRotations.setDouble(m_leftAngleEncoder.getPosition());
        rightAngleRotations.setDouble(m_rightAngleEncoder.getPosition());
        intakeSpeed.setDouble(intake.get());
        conveyorSpeed.setDouble(conveyor.get());
    }
}
