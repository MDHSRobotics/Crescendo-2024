package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{

    CANSparkMax shooter1;
    CANSparkMax shooter2;

    public Shooter(){
        shooter1 = new CANSparkMax(12, MotorType.kBrushless);
        shooter2 = new CANSparkMax(8, MotorType.kBrushless);
    }

    public void runMotors(double power){
        shooter1.set(-power);
        shooter2.set(power);
    }

    public Command runMotorsCommand() {
        // implicitly require `this`
        return this.runOnce(() -> runMotors(1));
    }

}
