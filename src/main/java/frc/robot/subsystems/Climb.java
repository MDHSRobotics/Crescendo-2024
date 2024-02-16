package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax climb1;
    CANSparkMax climb2;

    public Climb(){
        climb1 = new CANSparkMax(10, MotorType.kBrushless);
        climb2 = new CANSparkMax(11, MotorType.kBrushless);
 
    }

    public void runMotors(double climbPower){
        climb1.set(-climbPower);
        climb2.set(climbPower);
    }

}


