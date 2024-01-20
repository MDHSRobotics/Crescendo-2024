package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{

    CANSparkMax shooter1;
    CANSparkMax shooter2;

    public Shooter(){
        shooter1 = new CANSparkMax(4, MotorType.kBrushless);
        shooter2 = new CANSparkMax(3, MotorType.kBrushless);
        shooter2.setIdleMode(IdleMode.kCoast);
    }

    public void runMotors(double power){
        System.out.println(power);
        shooter1.set(-power);
        shooter2.set(power);
        SmartDashboard.putNumber("Shooter Power", power);
    }
    /*
    public Command runMotorsCommand() {
        // implicitly require `this`
        return this.startEnd(() -> runMotors(1),() -> runMotors(0));
    }*/

}
