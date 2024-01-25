package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooter1;
    private CANSparkMax shooter2;
    private CANSparkMax angle1;
    private CANSparkMax angle2;
    private CANSparkMax feeder;

    private SparkPIDController m_pidController;

    public Shooter(){
        shooter1 = new CANSparkMax(4, MotorType.kBrushless);
        shooter2 = new CANSparkMax(3, MotorType.kBrushless);
        angle1 = new CANSparkMax(5, MotorType.kBrushless);
        angle2 = new CANSparkMax(6, MotorType.kBrushless);
        feeder = new CANSparkMax(7, MotorType.kBrushless);

        m_pidController = angle1.getPIDController();
        m_pidController.setP(0.01);
        
        angle2.follow(angle1);
        //shooter2.setIdleMode(IdleMode.kCoast);
    }

    public void runShooter(double power, double feed){
        shooter1.set(-power);
        shooter2.set(power);
        SmartDashboard.putNumber("Shooter Power", power);
        feeder.set(feed);
    } 

    //adjust the angle of the shooter
    public void setAngle(double angle){
        m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Set Angle", angle);
    }

    public void shootSequence(){

    }

}
