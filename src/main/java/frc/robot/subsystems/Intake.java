package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private CANSparkFlex intake;
    private CANSparkMax conveyor;
    private CANSparkMax leftAngle;
    private CANSparkMax rightAngle;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_leftAngleEncoder;
    private RelativeEncoder m_rightAngleEncoder;

    private boolean m_calibration = false;

    /* Shuffleboard Values */
    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    private ShuffleboardLayout list = tab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3, 2);
    private GenericEntry angleRotations = list.add("Angle Rotations", 0.0).getEntry();
    private GenericEntry intakeSpeed = list.add("Intake Speed", 0.0).getEntry();
    private GenericEntry conveyorSpeed = list.add("Conveyer Speed", 0.0).getEntry();

    private GenericEntry intakeTopRotations =
      tab.add("Intake Top Rotations", 0.0)
        .withSize(2, 1)
        .getEntry();
    private GenericEntry intakeBottomRotations =
      tab.add("Intake Bottom Rotations", 34.0)
        .withSize(2, 1)
        .getEntry();
    private GenericEntry intakeMidPositions =
      tab.add("Intake Mid Rotations", 30.0)
        .withSize(2, 1)
        .getEntry();

    public Intake(){
        intake = new CANSparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
        conveyor = new CANSparkMax(IntakeConstants.kConveyorID, MotorType.kBrushless);
        leftAngle = new CANSparkMax(IntakeConstants.kLeftAngleID, MotorType.kBrushless);
        rightAngle = new CANSparkMax(IntakeConstants.kRightAngleID, MotorType.kBrushless);

        m_leftAngleEncoder = leftAngle.getEncoder();
        m_rightAngleEncoder = rightAngle.getEncoder();

        rightAngle.setIdleMode(IdleMode.kBrake);
        leftAngle.setIdleMode(IdleMode.kBrake);
        
        m_pidController = rightAngle.getPIDController();
        m_pidController.setP(0.1);

        // Consider increasing the period in different ways: https://docs.revrobotics.com/brushless/spark-max/control-interfaces#use-case-examples
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        leftAngle.setOpenLoopRampRate(0.1);
        rightAngle.setOpenLoopRampRate(0.1);
        intake.setOpenLoopRampRate(0.1);
        conveyor.setOpenLoopRampRate(0.1);

        leftAngle.follow(rightAngle, true);
    }

    

    public void topPosition(double power){
        if(!m_calibration){
            m_pidController.setReference(intakeTopRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
        }else{
            rightAngle.set(power);
        }
        
        intake.set(0.0);
        conveyor.set(0.0);
    }

    public void bottomPosition(){
        m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
        
        intake.set(1.0);
        conveyor.set(-1.0);
    }

    public void midPosition(){
        m_pidController.setReference(intakeMidPositions.getDouble(0), CANSparkMax.ControlType.kPosition);

        intake.set(1.0);
        conveyor.set(-1.0);
    }

    public void spitOut(){
        intake.set(-1);
        conveyor.set(1);
    }
    
    
    public void resetEncoders(){
        m_leftAngleEncoder.setPosition(0);
        m_rightAngleEncoder.setPosition(0);
    }

    public void setCalibration(){
        m_calibration = !m_calibration;
    }

    
    /* Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is disabled. */
    public void logData() {
        // Subsystem data
        angleRotations.setDouble(rightAngle.getEncoder().getPosition());
        intakeSpeed.setDouble(intake.get());
        conveyorSpeed.setDouble(conveyor.get());
    }
}
