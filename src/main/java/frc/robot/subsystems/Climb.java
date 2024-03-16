package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax leftClimb;
    CANSparkMax rightClimb;

    private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    private GenericEntry leftClimbRotations =
      tab.add("Left Climb Rotations", 0)
         .getEntry();
    private GenericEntry rightClimbRotations =
      tab.add("Right Climb Rotations", 0)
         .getEntry();


    public Climb(){
        leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);
        
        rightClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setIdleMode(IdleMode.kBrake);
    }

    public void runClimb(double climb1Power, double climb2Power){  
        leftClimb.set(-climb1Power);
        rightClimb.set(climb2Power);

        /* Logging */
        leftClimbRotations.setDouble(leftClimb.getEncoder().getPosition());
        rightClimbRotations.setDouble(rightClimb.getEncoder().getPosition());
    }

}


