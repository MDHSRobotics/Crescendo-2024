package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private CANSparkFlex intake;
    private CANSparkMax conveyor;
    private CANSparkMax leftAngle;
    private CANSparkMax rightAngle;

    private SparkPIDController m_pidController;

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private GenericEntry intakeTopRotations =
      tab.add("Intake Top Rotations", 0.0)
        .getEntry();
    private GenericEntry intakeBottomRotations =
      tab.add("Intake Bottom Rotations", 34.0)
        .getEntry();
    private GenericEntry intakeRotations =
      tab.add("Intake Rotations", -20.0)
        .getEntry();
    private GenericEntry intakeMidPositions =
      tab.add("Intake Mid Rotations", 30.0)
        .getEntry();
    private GenericEntry spitPower =
      tab.add("Spit Power", -1.0)
        .getEntry();

    private boolean m_calibration = false;

    public Intake(){
        intake = new CANSparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
        conveyor = new CANSparkMax(IntakeConstants.kConveyorID, MotorType.kBrushless);
        leftAngle = new CANSparkMax(IntakeConstants.kLeftAngleID, MotorType.kBrushless);
        rightAngle = new CANSparkMax(IntakeConstants.kRightAngleID, MotorType.kBrushless);

        rightAngle.setIdleMode(IdleMode.kBrake);
        leftAngle.setIdleMode(IdleMode.kBrake);
        
        m_pidController = rightAngle.getPIDController();
        m_pidController.setP(0.1);

        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        rightAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);
        conveyor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

        leftAngle.setOpenLoopRampRate(0.2);
        rightAngle.setOpenLoopRampRate(0.2);
        intake.setOpenLoopRampRate(0.2);
        conveyor.setOpenLoopRampRate(0.2);

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

        /* Logging */
        intakeRotations.setDouble(rightAngle.getEncoder().getPosition());
    }

    public void bottomPosition(){
        m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
        
        intake.set(1.0);
        conveyor.set(-1.0);

        /* Logging */
        intakeRotations.setDouble(rightAngle.getEncoder().getPosition());
    }

    public void midPosition(){
        m_pidController.setReference(intakeMidPositions.getDouble(0), CANSparkMax.ControlType.kPosition);

        intake.set(1.0);
        conveyor.set(-1.0);
        
        /* Logging */
        intakeRotations.setDouble(rightAngle.getEncoder().getPosition());
    }

    public void spitOut(){
        intake.set(spitPower.getDouble(-1));
        conveyor.set(1);
    }
    
    
    public void resetEncoders(){
        leftAngle.getEncoder().setPosition(0);
        rightAngle.getEncoder().setPosition(0);
    }

    public void setCalibration(){
        m_calibration = !m_calibration;
    }
}
