package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //private CANSparkMax mainIntake;
    //private CANSparkMax conveyor;

    public Intake(){
        //mainIntake = new CANSparkMax(10, MotorType.kBrushless);
       // conveyor = new CANSparkMax(11, MotorType.kBrushless);
    }

    public void runIntake(double power){
       // mainIntake.set(power);
        //conveyor.set(power);
        SmartDashboard.putNumber("Intake Power", power);
    }
    
}
