package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax climb1;
    CANSparkMax climb2;

    public Climb(){
        climb1 = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        climb2 = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);
 
    }

    public void runMotors(double climb1Power, double climb2Power){
        climb1.set(-climb1Power);
        climb2.set(climb2Power);
        SmartDashboard.putNumber("Climb Power", climb1Power);
    }

}


