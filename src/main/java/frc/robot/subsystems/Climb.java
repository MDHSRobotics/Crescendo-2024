package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax leftClimb;
    CANSparkMax rightClimb;


    public Climb(){
        leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);
        
        rightClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setIdleMode(IdleMode.kBrake);
    }

    public void runClimb(double climb1Power, double climb2Power){  
        leftClimb.set(-climb1Power);
        rightClimb.set(climb2Power);
    }


    // Initialize the Sendable that will log values to Shuffleboard in a nice little table for us
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Left Motor Rotations", () -> leftClimb.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Right Motor Rotations", () -> rightClimb.getEncoder().getPosition(), null);
    }

}


