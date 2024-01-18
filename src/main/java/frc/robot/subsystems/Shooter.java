package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{

    CANSparkMax shooter1;
    CANSparkMax shooter2;

    public Shooter(){
        shooter1 = new CANSparkMax(12, MotorType.kBrushless);
        shooter2 = new CANSparkMax(8, MotorType.kBrushless);
        shooter2.setIdleMode(IdleMode.kCoast);
    }

    public void runMotors(double power){
        System.out.println(power);
        shooter1.set(-power);
        shooter2.set(power);
    }
    /*
    public Command runMotorsCommand() {
        // implicitly require `this`
        return this.startEnd(() -> runMotors(1),() -> runMotors(0));
    }*/

}
