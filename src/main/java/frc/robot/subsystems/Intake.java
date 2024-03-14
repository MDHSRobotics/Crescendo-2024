package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      tab.add("Intake Bottom Rotations", -20.0)
        .getEntry();
    private GenericEntry intakeRotations =
      tab.add("Intake Rotations", -20.0)
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
        m_pidController.setP(0.03);

        leftAngle.follow(rightAngle, true);
    }

    public void runIntake(double position, double power){
        intake.set(power);
        conveyor.set(-power);

        rightAngle.set(position * 0.2);

        SmartDashboard.putNumber("Intake Power", power);
        SmartDashboard.putNumber("Intake Position", position);
    }

    public void topPosition(double power){
        if(!m_calibration){
            m_pidController.setReference(intakeTopRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
        }else{
            rightAngle.set(power);
        }
        
        intakeRotations.setDouble(rightAngle.getEncoder().getPosition());

        intake.set(0);
        conveyor.set(0);
    }

    public void bottomPosition(){
        m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
        intake.set(1);
        conveyor.set(-0.5);

    }

    public void spitOut(){
        intake.set(-1);
        conveyor.set(1);
    }
    
    
    public void resetEncoders(){
        leftAngle.getEncoder().setPosition(0);
        rightAngle.getEncoder().setPosition(0);
    }

    public void setCalibration(boolean mode){
        m_calibration = mode;
    }
}
